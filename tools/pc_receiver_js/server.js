const dgram = require('node:dgram');
const http = require('node:http');
const fs = require('node:fs');
const net = require('node:net');
const path = require('node:path');

const MAGIC = 0x56535544; // VSUD
const HEADER_SIZE = 20;
const PUBLIC_DIR = path.join(__dirname, 'public');
const RECORDINGS_DIR = path.join(__dirname, 'recordings');

const BIND_HOST = process.env.BIND_HOST || '0.0.0.0';
const UDP_PORT = Number(process.env.UDP_PORT || 10000);
const TCP_PORT = Number(process.env.TCP_PORT || 10001);
const HTTP_PORT = Number(process.env.HTTP_PORT || 8080);

const latestByMode = {
  0: { jpeg: null, frameId: -1, updatedAtMs: 0 }, // binary
  1: { jpeg: null, frameId: -1, updatedAtMs: 0 }  // gray
};
let latestStatus = { message: 'waiting' };

const inflightFrames = new Map();

function ensureRecordingsDir() {
  fs.mkdirSync(RECORDINGS_DIR, { recursive: true });
}

function sanitizeFolderName(name) {
  if (typeof name !== 'string') return '';
  const trimmed = name.trim();
  if (!trimmed) return '';
  if (!/^[A-Za-z0-9._-]+$/.test(trimmed)) return '';
  return trimmed;
}

function folderPathFromName(name) {
  const safe = sanitizeFolderName(name);
  if (!safe) return '';
  return path.join(RECORDINGS_DIR, safe);
}

function sendJson(res, code, obj) {
  res.writeHead(code, {
    'Content-Type': 'application/json; charset=utf-8',
    'Cache-Control': 'no-store'
  });
  res.end(JSON.stringify(obj));
}

function readJsonBody(req, limitBytes = 300 * 1024 * 1024) {
  return new Promise((resolve, reject) => {
    let size = 0;
    const chunks = [];
    req.on('data', (chunk) => {
      size += chunk.length;
      if (size > limitBytes) {
        reject(new Error('payload too large'));
        req.destroy();
        return;
      }
      chunks.push(chunk);
    });
    req.on('end', () => {
      try {
        const text = Buffer.concat(chunks).toString('utf8');
        resolve(text ? JSON.parse(text) : {});
      } catch (err) {
        reject(err);
      }
    });
    req.on('error', reject);
  });
}

function extByMime(mime) {
  if (mime === 'video/webm' || mime === 'video/webm;codecs=vp8' || mime === 'video/webm;codecs=vp9') return '.webm';
  if (mime === 'application/json') return '.json';
  return '.bin';
}

function writeRecordingFiles(folderName, payload) {
  const folderPath = folderPathFromName(folderName);
  if (!folderPath) throw new Error('invalid folder name');
  fs.mkdirSync(folderPath, { recursive: true });

  const statusPayload = {
    recorded_at_ms: payload.recorded_at_ms,
    duration_ms: payload.duration_ms,
    frame_count: payload.frame_count,
    statuses: Array.isArray(payload.statuses) ? payload.statuses : []
  };
  fs.writeFileSync(path.join(folderPath, 'status.json'), JSON.stringify(statusPayload, null, 2), 'utf8');

  const videos = payload.videos && typeof payload.videos === 'object' ? payload.videos : {};
  const videoMeta = {};
  const keys = ['gray', 'binary', 'ipm', 'ipm_raw', 'curvature'];
  for (const key of keys) {
    const item = videos[key];
    if (!item || typeof item.data_b64 !== 'string' || !item.data_b64) continue;
    const mime = (typeof item.mime === 'string' && item.mime) ? item.mime : 'video/webm';
    const ext = extByMime(mime);
    const fileName = `${key}${ext}`;
    const filePath = path.join(folderPath, fileName);
    fs.writeFileSync(filePath, Buffer.from(item.data_b64, 'base64'));
    videoMeta[key] = { file: fileName, mime };
  }

  const meta = {
    folder: folderName,
    saved_at_ms: Date.now(),
    recorded_at_ms: payload.recorded_at_ms,
    duration_ms: payload.duration_ms,
    frame_count: payload.frame_count,
    videos: videoMeta
  };
  fs.writeFileSync(path.join(folderPath, 'meta.json'), JSON.stringify(meta, null, 2), 'utf8');
  return meta;
}

function listRecordingFolders() {
  ensureRecordingsDir();
  const entries = fs.readdirSync(RECORDINGS_DIR, { withFileTypes: true })
    .filter((entry) => entry.isDirectory())
    .map((entry) => {
      const fullPath = path.join(RECORDINGS_DIR, entry.name);
      let mtimeMs = 0;
      try {
        mtimeMs = fs.statSync(fullPath).mtimeMs;
      } catch (_) {
        mtimeMs = 0;
      }
      return { folder: entry.name, mtime_ms: Math.round(mtimeMs) };
    })
    .sort((a, b) => b.mtime_ms - a.mtime_ms);
  return entries;
}

function parseHeader(buf) {
  if (buf.length < HEADER_SIZE) return null;
  const magic = buf.readUInt32BE(0);
  const frameId = buf.readUInt32BE(4);
  const chunkIdx = buf.readUInt16BE(8);
  const chunkTotal = buf.readUInt16BE(10);
  const payloadLen = buf.readUInt16BE(12);
  const width = buf.readUInt16BE(14);
  const height = buf.readUInt16BE(16);
  const mode = buf.readUInt8(18);
  return { magic, frameId, chunkIdx, chunkTotal, payloadLen, width, height, mode };
}

function cleanupInflight() {
  const now = Date.now();
  for (const [frameId, entry] of inflightFrames.entries()) {
    if (now - entry.ts > 2000) {
      inflightFrames.delete(frameId);
    }
  }
}

function isFrameNewer(prevFrameId, nextFrameId) {
  // 按 uint32 序关系判断 next 是否比 prev 更新，兼容回绕。
  const diff = ((nextFrameId >>> 0) - (prevFrameId >>> 0)) >>> 0;
  return diff !== 0 && diff < 0x80000000;
}

function shouldAcceptFrame(mode, frameId, nowMs) {
  const latest = latestByMode[mode];
  if (!latest) return false;
  if (latest.frameId < 0) return true;
  if (isFrameNewer(latest.frameId, frameId)) return true;

  // 主板重连/发送中断后，frameId 可能从 0 重新开始。
  // 当该 mode 超过 1.5s 没有新图后，允许接纳“非更新序”帧作为新起点。
  if ((nowMs - latest.updatedAtMs) > 1500) return true;
  return false;
}

function onUdpMessage(msg) {
  const hdr = parseHeader(msg);
  if (!hdr) return;
  if (hdr.magic !== MAGIC) return;
  if (hdr.chunkTotal === 0 || hdr.chunkIdx >= hdr.chunkTotal) return;
  if (HEADER_SIZE + hdr.payloadLen > msg.length) return;

  let entry = inflightFrames.get(hdr.frameId);
  if (!entry) {
    entry = {
      chunkTotal: hdr.chunkTotal,
      chunks: new Map(),
      width: hdr.width,
      height: hdr.height,
      mode: hdr.mode,
      ts: Date.now()
    };
    inflightFrames.set(hdr.frameId, entry);
  }

  const payload = msg.subarray(HEADER_SIZE, HEADER_SIZE + hdr.payloadLen);
  entry.chunks.set(hdr.chunkIdx, payload);
  entry.ts = Date.now();

  if (entry.chunks.size === entry.chunkTotal) {
    const ordered = [];
    for (let i = 0; i < entry.chunkTotal; i += 1) {
      const chunk = entry.chunks.get(i);
      if (!chunk) {
        return;
      }
      ordered.push(chunk);
    }
    const jpeg = Buffer.concat(ordered);
    const nowMs = Date.now();
    if (shouldAcceptFrame(hdr.mode, hdr.frameId, nowMs)) {
      latestByMode[hdr.mode].frameId = hdr.frameId >>> 0;
      latestByMode[hdr.mode].jpeg = jpeg;
      latestByMode[hdr.mode].updatedAtMs = nowMs;
    }
    inflightFrames.delete(hdr.frameId);
  }
}

function startUdpReceiver() {
  const udp = dgram.createSocket('udp4');
  udp.on('message', onUdpMessage);
  udp.bind(UDP_PORT, BIND_HOST, () => {
    console.log(`[JS_RECEIVER] UDP video listening on ${BIND_HOST}:${UDP_PORT}`);
  });
  setInterval(cleanupInflight, 500);
}

function startTcpReceiver() {
  const server = net.createServer((socket) => {
    let buffer = '';
    socket.setEncoding('utf8');

    socket.on('data', (chunk) => {
      buffer += chunk;
      let idx = buffer.indexOf('\n');
      while (idx >= 0) {
        const line = buffer.slice(0, idx).trim();
        buffer = buffer.slice(idx + 1);
        if (line) {
          try {
            latestStatus = JSON.parse(line);
          } catch (_) {
            // ignore malformed lines
          }
        }
        idx = buffer.indexOf('\n');
      }
    });
  });

  server.listen(TCP_PORT, BIND_HOST, () => {
    console.log(`[JS_RECEIVER] TCP status listening on ${BIND_HOST}:${TCP_PORT}`);
  });
}

function serveFile(res, filePath, contentType) {
  fs.readFile(filePath, (err, data) => {
    if (err) {
      res.writeHead(404, { 'Content-Type': 'text/plain; charset=utf-8' });
      res.end('not found');
      return;
    }
    res.writeHead(200, {
      'Content-Type': contentType,
      'Cache-Control': 'no-store'
    });
    res.end(data);
  });
}

function startHttpServer() {
  ensureRecordingsDir();
  const server = http.createServer((req, res) => {
    const reqUrl = new URL(req.url || '/', `http://${req.headers.host || 'localhost'}`);
    const pathname = reqUrl.pathname;

    if (pathname === '/' || pathname === '/index.html') {
      serveFile(res, path.join(PUBLIC_DIR, 'index.html'), 'text/html; charset=utf-8');
      return;
    }

    if (pathname === '/api/status') {
      const payload = JSON.stringify(latestStatus);
      res.writeHead(200, {
        'Content-Type': 'application/json; charset=utf-8',
        'Cache-Control': 'no-store'
      });
      res.end(payload);
      return;
    }

    if (pathname === '/api/frame_gray.jpg') {
      const latestGray = latestByMode[1].jpeg;
      if (!latestGray) {
        res.writeHead(503, {
          'Content-Type': 'text/plain; charset=utf-8',
          'Cache-Control': 'no-store'
        });
        res.end('gray frame not ready');
        return;
      }
      res.writeHead(200, {
        'Content-Type': 'image/jpeg',
        'Cache-Control': 'no-store'
      });
      res.end(latestGray);
      return;
    }

    if (pathname === '/api/frame_binary.jpg') {
      const latestBinary = latestByMode[0].jpeg;
      if (!latestBinary) {
        res.writeHead(503, {
          'Content-Type': 'text/plain; charset=utf-8',
          'Cache-Control': 'no-store'
        });
        res.end('binary frame not ready');
        return;
      }
      res.writeHead(200, {
        'Content-Type': 'image/jpeg',
        'Cache-Control': 'no-store'
      });
      res.end(latestBinary);
      return;
    }

    if (pathname === '/api/recordings/save' && req.method === 'POST') {
      readJsonBody(req).then((payload) => {
        const folderHint = sanitizeFolderName(payload.folder || '');
        const folder = folderHint || `vision_record_${Date.now()}`;
        const meta = writeRecordingFiles(folder, payload || {});
        sendJson(res, 200, { ok: true, folder, meta });
      }).catch((err) => {
        sendJson(res, 400, { ok: false, error: String(err && err.message ? err.message : err) });
      });
      return;
    }

    if (pathname === '/api/recordings/list') {
      sendJson(res, 200, { ok: true, folders: listRecordingFolders() });
      return;
    }

    if (pathname === '/api/recordings/load') {
      const folder = sanitizeFolderName(reqUrl.searchParams.get('folder') || '');
      const folderPath = folderPathFromName(folder);
      if (!folder || !folderPath || !fs.existsSync(folderPath)) {
        sendJson(res, 404, { ok: false, error: 'folder not found' });
        return;
      }

      let status = null;
      let meta = null;
      try {
        const statusPath = path.join(folderPath, 'status.json');
        if (fs.existsSync(statusPath)) {
          status = JSON.parse(fs.readFileSync(statusPath, 'utf8'));
        }
        const metaPath = path.join(folderPath, 'meta.json');
        if (fs.existsSync(metaPath)) {
          meta = JSON.parse(fs.readFileSync(metaPath, 'utf8'));
        }
      } catch (err) {
        sendJson(res, 500, { ok: false, error: String(err && err.message ? err.message : err) });
        return;
      }

      const videos = {};
      const files = fs.readdirSync(folderPath);
      const keys = ['gray', 'binary', 'ipm', 'ipm_raw', 'curvature'];
      for (const key of keys) {
        const hit = files.find((name) => name === `${key}.webm` || name === `${key}.mp4`);
        if (hit) videos[key] = `/api/recordings/file?folder=${encodeURIComponent(folder)}&name=${encodeURIComponent(hit)}`;
      }
      sendJson(res, 200, { ok: true, folder, status, meta, videos });
      return;
    }

    if (pathname === '/api/recordings/file') {
      const folder = sanitizeFolderName(reqUrl.searchParams.get('folder') || '');
      const name = path.basename(reqUrl.searchParams.get('name') || '');
      if (!folder || !name) {
        res.writeHead(400, { 'Content-Type': 'text/plain; charset=utf-8' });
        res.end('bad request');
        return;
      }
      if (!/^[A-Za-z0-9._-]+$/.test(name)) {
        res.writeHead(400, { 'Content-Type': 'text/plain; charset=utf-8' });
        res.end('bad file name');
        return;
      }

      const folderPath = folderPathFromName(folder);
      if (!folderPath || !fs.existsSync(folderPath)) {
        res.writeHead(404, { 'Content-Type': 'text/plain; charset=utf-8' });
        res.end('folder not found');
        return;
      }
      const filePath = path.join(folderPath, name);
      if (!filePath.startsWith(folderPath) || !fs.existsSync(filePath)) {
        res.writeHead(404, { 'Content-Type': 'text/plain; charset=utf-8' });
        res.end('file not found');
        return;
      }

      const ext = path.extname(name).toLowerCase();
      const contentType = ext === '.webm'
        ? 'video/webm'
        : (ext === '.mp4'
          ? 'video/mp4'
          : (ext === '.json' ? 'application/json; charset=utf-8' : 'application/octet-stream'));
      res.writeHead(200, {
        'Content-Type': contentType,
        'Cache-Control': 'no-store'
      });
      fs.createReadStream(filePath).pipe(res);
      return;
    }

    if (pathname === '/healthz') {
      res.writeHead(200, { 'Content-Type': 'text/plain; charset=utf-8' });
      res.end('ok');
      return;
    }

    res.writeHead(404, { 'Content-Type': 'text/plain; charset=utf-8' });
    res.end('not found');
  });

  server.listen(HTTP_PORT, BIND_HOST, () => {
    console.log(`[JS_RECEIVER] HTTP web listening on http://${BIND_HOST}:${HTTP_PORT}/`);
  });
}

startUdpReceiver();
startTcpReceiver();
startHttpServer();
