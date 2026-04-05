const dgram = require('node:dgram');
const http = require('node:http');
const fs = require('node:fs');
const net = require('node:net');
const path = require('node:path');
const WebSocket = require('ws');
const { summarizeSyncStatus } = require('../pc_receiver_local_compute/wasm_sync_meta.js');

const MAGIC = 0x56535544; // VSUD
const HEADER_SIZE = 20;
const PUBLIC_DIR = path.join(__dirname, 'public');
const RECORDINGS_DIR = path.join(__dirname, 'recordings');
const WASM_DIR = path.join(PUBLIC_DIR, 'wasm');
const ROOT_DIR = path.join(__dirname, '..', '..');
const WASM_SYNC_METADATA_PATH = path.join(WASM_DIR, 'vision_pipeline.sync.json');

const BIND_HOST = process.env.BIND_HOST || '0.0.0.0';
const UDP_PORT = Number(process.env.UDP_PORT || 10000);
const TCP_PORT = Number(process.env.TCP_PORT || 10001);
const HTTP_PORT = Number(process.env.HTTP_PORT || 8080);
const WEB_IMAGE_FORMAT_JPEG = 0;
const WEB_IMAGE_FORMAT_PNG = 1;
const WEB_IMAGE_FORMAT_BMP = 2;

const latestByMode = {
  0: { image: null, frameId: -1, updatedAtMs: 0, width: 0, height: 0, mode: 0, format: WEB_IMAGE_FORMAT_JPEG },
  1: { image: null, frameId: -1, updatedAtMs: 0, width: 0, height: 0, mode: 1, format: WEB_IMAGE_FORMAT_JPEG },
  2: { image: null, frameId: -1, updatedAtMs: 0, width: 0, height: 0, mode: 2, format: WEB_IMAGE_FORMAT_JPEG }
};
let latestStatus = { message: 'waiting' };

const inflightFrames = new Map();
const udpByteEvents = [];
const udpFrameEvents = [];
let cachedWasmSyncStatus = null;
let cachedWasmSyncStatusAtMs = 0;

let wss = null;

function broadcastWs(type, data) {
  if (!wss) return;
  const txt = JSON.stringify({ type, data, ts: Date.now() });
  for (const client of wss.clients) {
    if (client.readyState === WebSocket.OPEN) {
      client.send(txt);
    }
  }
}

function modeName(mode) {
  if (mode === 0) return 'binary';
  if (mode === 1) return 'gray';
  if (mode === 2) return 'rgb';
  return `mode_${mode}`;
}

function toBool01(value) {
  return value === true || value === 1 || value === '1';
}

function round3(value) {
  return Number.isFinite(value) ? Math.round(value * 1000) / 1000 : null;
}

function getWasmSyncStatus() {
  const now = Date.now();
  if (cachedWasmSyncStatus && (now - cachedWasmSyncStatusAtMs) < 2000) {
    return cachedWasmSyncStatus;
  }
  cachedWasmSyncStatus = summarizeSyncStatus(ROOT_DIR, WASM_SYNC_METADATA_PATH);
  cachedWasmSyncStatusAtMs = now;
  return cachedWasmSyncStatus;
}

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
    video_labels: (payload.video_labels && typeof payload.video_labels === 'object') ? payload.video_labels : {},
    statuses: Array.isArray(payload.statuses) ? payload.statuses : []
  };
  fs.writeFileSync(path.join(folderPath, 'status.json'), JSON.stringify(statusPayload, null, 2), 'utf8');

  const videos = payload.videos && typeof payload.videos === 'object' ? payload.videos : {};
  const videoMeta = {};
  for (const key of Object.keys(videos)) {
    if (!/^[A-Za-z0-9._-]+$/.test(key)) continue;
    const item = videos[key];
    if (!item || typeof item.data_b64 !== 'string' || !item.data_b64) continue;
    const mime = (typeof item.mime === 'string' && item.mime) ? item.mime : 'video/webm';
    const ext = extByMime(mime);
    const fileName = `${key}${ext}`;
    const filePath = path.join(folderPath, fileName);
    fs.writeFileSync(filePath, Buffer.from(item.data_b64, 'base64'));
    const title = (typeof item.title === 'string' && item.title.trim()) ? item.title.trim() : key;
    videoMeta[key] = { file: fileName, mime, title };
  }

  const meta = {
    folder: folderName,
    saved_at_ms: Date.now(),
    recorded_at_ms: payload.recorded_at_ms,
    duration_ms: payload.duration_ms,
    frame_count: payload.frame_count,
    video_labels: statusPayload.video_labels,
    videos: videoMeta
  };
  fs.writeFileSync(path.join(folderPath, 'meta.json'), JSON.stringify(meta, null, 2), 'utf8');
  return meta;
}

function listRecordingFolders() {
  ensureRecordingsDir();
  return fs.readdirSync(RECORDINGS_DIR, { withFileTypes: true })
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
  const format = buf.readUInt8(19);
  return { magic, frameId, chunkIdx, chunkTotal, payloadLen, width, height, mode, format };
}

function sanitizeImageFormat(format) {
  if (format === WEB_IMAGE_FORMAT_PNG) return WEB_IMAGE_FORMAT_PNG;
  if (format === WEB_IMAGE_FORMAT_BMP) return WEB_IMAGE_FORMAT_BMP;
  return WEB_IMAGE_FORMAT_JPEG;
}

function contentTypeByImageFormat(format) {
  if (format === WEB_IMAGE_FORMAT_PNG) return 'image/png';
  if (format === WEB_IMAGE_FORMAT_BMP) return 'image/bmp';
  return 'image/jpeg';
}

function cleanupInflight() {
  const now = Date.now();
  for (const [frameId, entry] of inflightFrames.entries()) {
    if (now - entry.ts > 2000) {
      inflightFrames.delete(frameId);
    }
  }
  while (udpByteEvents.length > 0 && now - udpByteEvents[0].ts > 5000) udpByteEvents.shift();
  while (udpFrameEvents.length > 0 && now - udpFrameEvents[0].ts > 5000) udpFrameEvents.shift();
}

function isFrameNewer(prevFrameId, nextFrameId) {
  const diff = ((nextFrameId >>> 0) - (prevFrameId >>> 0)) >>> 0;
  return diff !== 0 && diff < 0x80000000;
}

function isLikelyFrameCounterReset(prevFrameId, nextFrameId) {
  const prev = prevFrameId >>> 0;
  const next = nextFrameId >>> 0;
  // 主板重启/重刷程序后，frame_id 常常会重新从很小的值开始。
  // 这里把“上一帧已经跑到较大值，而新帧突然回到很小值”视为复位信号，立即接纳。
  if (prev < 1024) return false;
  return next < 64;
}

function shouldAcceptFrame(mode, frameId, nowMs) {
  const latest = latestByMode[mode];
  if (!latest) return false;
  if (latest.frameId < 0) return true;
  if (isFrameNewer(latest.frameId, frameId)) return true;
  if (isLikelyFrameCounterReset(latest.frameId, frameId)) return true;
  if ((nowMs - latest.updatedAtMs) > 1500) return true;
  return false;
}

function onUdpMessage(msg) {
  udpByteEvents.push({ ts: Date.now(), bytes: msg.length });
  const hdr = parseHeader(msg);
  if (!hdr) return;
  if (hdr.magic !== MAGIC) return;
  if (!(hdr.mode in latestByMode)) return;
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
      format: sanitizeImageFormat(hdr.format),
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
    const image = Buffer.concat(ordered);
    const nowMs = Date.now();
    if (shouldAcceptFrame(hdr.mode, hdr.frameId, nowMs)) {
      const wireBytes = ordered.reduce((sum, chunk) => sum + chunk.length, 0) + (entry.chunkTotal * HEADER_SIZE);
      latestByMode[hdr.mode] = {
        image,
        frameId: hdr.frameId >>> 0,
        updatedAtMs: nowMs,
        width: hdr.width,
        height: hdr.height,
        mode: hdr.mode,
        format: sanitizeImageFormat(hdr.format),
        wireBytes
      };
      
      broadcastWs('frame', {
        mode: hdr.mode,
        frameId: hdr.frameId >>> 0,
        width: hdr.width,
        height: hdr.height,
        format: sanitizeImageFormat(hdr.format)
      });
      
      udpFrameEvents.push({ ts: nowMs, mode: hdr.mode, wireBytes });
    }
    inflightFrames.delete(hdr.frameId);
  }
}

function buildTransportTelemetry(status) {
  const now = Date.now();
  const oneSecAgo = now - 1000;
  const recentBytes = udpByteEvents.filter((item) => item.ts >= oneSecAgo);
  const recentFrames = udpFrameEvents.filter((item) => item.ts >= oneSecAgo);

  const rxBytesPerSec = recentBytes.reduce((sum, item) => sum + item.bytes, 0);
  const rxFramesPerSec = recentFrames.length;
  const rxFramesByMode = { gray: 0, binary: 0, rgb: 0 };
  const frameBytesByMode = { gray: 0, binary: 0, rgb: 0 };
  for (const item of recentFrames) {
    const key = modeName(item.mode);
    if (key in rxFramesByMode) {
      rxFramesByMode[key] += 1;
      frameBytesByMode[key] += item.wireBytes;
    }
  }

  const avgWireBytesByMode = { gray: null, binary: null, rgb: null };
  for (const key of Object.keys(avgWireBytesByMode)) {
    if (rxFramesByMode[key] > 0) {
      avgWireBytesByMode[key] = Math.round(frameBytesByMode[key] / rxFramesByMode[key]);
    }
  }

  const maxFps = Number(status && status.udp_web_max_fps);
  const peakContributors = [];
  const activeModeMissingRecentFrame = [];
  if (toBool01(status && status.udp_web_send_gray) && Number.isFinite(avgWireBytesByMode.gray)) {
    peakContributors.push(avgWireBytesByMode.gray);
  } else if (toBool01(status && status.udp_web_send_gray)) {
    activeModeMissingRecentFrame.push('gray');
  }
  if (toBool01(status && status.udp_web_send_binary) && Number.isFinite(avgWireBytesByMode.binary)) {
    peakContributors.push(avgWireBytesByMode.binary);
  } else if (toBool01(status && status.udp_web_send_binary)) {
    activeModeMissingRecentFrame.push('binary');
  }
  if (toBool01(status && status.udp_web_send_rgb) && Number.isFinite(avgWireBytesByMode.rgb)) {
    peakContributors.push(avgWireBytesByMode.rgb);
  } else if (toBool01(status && status.udp_web_send_rgb)) {
    activeModeMissingRecentFrame.push('rgb');
  }
  const estimatedPeakBytesPerSec =
    Number.isFinite(maxFps) && maxFps > 0 && peakContributors.length > 0 && activeModeMissingRecentFrame.length === 0
      ? Math.round(maxFps * peakContributors.reduce((sum, value) => sum + value, 0))
      : null;
  const observedUtilization =
    Number.isFinite(estimatedPeakBytesPerSec) && estimatedPeakBytesPerSec > 0
      ? round3(rxBytesPerSec / estimatedPeakBytesPerSec)
      : null;

  return {
    rx_udp_bytes_per_sec: rxBytesPerSec,
    rx_udp_kib_per_sec: round3(rxBytesPerSec / 1024),
    rx_udp_mbps: round3((rxBytesPerSec * 8) / 1000000),
    rx_udp_frames_per_sec: rxFramesPerSec,
    rx_udp_gray_fps: rxFramesByMode.gray,
    rx_udp_binary_fps: rxFramesByMode.binary,
    rx_udp_rgb_fps: rxFramesByMode.rgb,
    rx_udp_avg_gray_frame_bytes: avgWireBytesByMode.gray,
    rx_udp_avg_binary_frame_bytes: avgWireBytesByMode.binary,
    rx_udp_avg_rgb_frame_bytes: avgWireBytesByMode.rgb,
    rx_udp_estimated_peak_bytes_per_sec: estimatedPeakBytesPerSec,
    rx_udp_estimated_peak_kib_per_sec: Number.isFinite(estimatedPeakBytesPerSec) ? round3(estimatedPeakBytesPerSec / 1024) : null,
    rx_udp_estimated_peak_mbps: Number.isFinite(estimatedPeakBytesPerSec) ? round3((estimatedPeakBytesPerSec * 8) / 1000000) : null,
    rx_udp_utilization_ratio: observedUtilization,
    rx_udp_estimated_peak_note: Number.isFinite(estimatedPeakBytesPerSec)
      ? '仅按最近1秒真实收到的活跃图像流估算满载，不再使用旧帧回退值'
      : (activeModeMissingRecentFrame.length > 0
        ? `当前启用的 ${activeModeMissingRecentFrame.join('/')} 最近1秒没有有效帧，暂不估算满载`
        : '当前 max_fps=0 或最近窗口没有有效图像帧，无法估算满载')
  };
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
            broadcastWs('status', latestStatus);
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

function writeImage(res, frame, notReadyMessage) {
  if (!frame || !frame.image) {
    res.writeHead(503, {
      'Content-Type': 'text/plain; charset=utf-8',
      'Cache-Control': 'no-store'
    });
    res.end(notReadyMessage);
    return;
  }
  res.writeHead(200, {
    'Content-Type': contentTypeByImageFormat(frame.format),
    'Cache-Control': 'no-store'
  });
  res.end(frame.image);
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
    if (pathname === '/local_compute.html') {
      serveFile(res, path.join(PUBLIC_DIR, 'local_compute.html'), 'text/html; charset=utf-8');
      return;
    }
    if (pathname === '/local_compute_app.js') {
      serveFile(res, path.join(PUBLIC_DIR, 'local_compute_app.js'), 'application/javascript; charset=utf-8');
      return;
    }
    if (pathname === '/shared_receiver_core.js') {
      serveFile(res, path.join(PUBLIC_DIR, 'shared_receiver_core.js'), 'application/javascript; charset=utf-8');
      return;
    }
    if (pathname === '/pipeline_worker.js') {
      serveFile(res, path.join(PUBLIC_DIR, 'pipeline_worker.js'), 'application/javascript; charset=utf-8');
      return;
    }
    if (pathname.startsWith('/wasm/')) {
      const rel = pathname.slice('/wasm/'.length);
      const safeName = path.basename(rel);
      const filePath = path.join(WASM_DIR, safeName);
      const ext = path.extname(safeName).toLowerCase();
      const contentType = ext === '.js'
        ? 'application/javascript; charset=utf-8'
        : (ext === '.wasm' ? 'application/wasm' : 'application/octet-stream');
      serveFile(res, filePath, contentType);
      return;
    }

    if (pathname === '/api/status') {
      const payload = JSON.stringify(Object.assign({}, latestStatus, buildTransportTelemetry(latestStatus)));
      res.writeHead(200, {
        'Content-Type': 'application/json; charset=utf-8',
        'Cache-Control': 'no-store'
      });
      res.end(payload);
      return;
    }

    if (pathname === '/api/wasm_sync_status') {
      sendJson(res, 200, getWasmSyncStatus());
      return;
    }

    if (pathname === '/api/frame_meta') {
      sendJson(res, 200, {
        gray: latestByMode[1],
        binary: latestByMode[0],
        rgb: latestByMode[2]
      });
      return;
    }

    if (pathname === '/api/frame_gray.jpg') {
      writeImage(res, latestByMode[1], 'gray frame not ready');
      return;
    }

    if (pathname === '/api/frame_binary.jpg') {
      writeImage(res, latestByMode[0], 'binary frame not ready');
      return;
    }

    if (pathname === '/api/frame_rgb.jpg') {
      writeImage(res, latestByMode[2], 'rgb frame not ready');
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
      if (meta && meta.videos && typeof meta.videos === 'object') {
        for (const [key, item] of Object.entries(meta.videos)) {
          if (!item || typeof item.file !== 'string' || !item.file) continue;
          videos[key] = `/api/recordings/file?folder=${encodeURIComponent(folder)}&name=${encodeURIComponent(item.file)}`;
        }
      } else {
        const files = fs.readdirSync(folderPath);
        for (const name of files) {
          const ext = path.extname(name).toLowerCase();
          if (ext !== '.webm' && ext !== '.mp4') continue;
          const key = path.basename(name, ext);
          if (!/^[A-Za-z0-9._-]+$/.test(key)) continue;
          videos[key] = `/api/recordings/file?folder=${encodeURIComponent(folder)}&name=${encodeURIComponent(name)}`;
        }
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

  wss = new WebSocket.Server({ server });
  wss.on('connection', (ws) => {
    // Send immediate state upon connection
    ws.send(JSON.stringify({ type: 'status', data: latestStatus, ts: Date.now() }));
  });

  server.listen(HTTP_PORT, BIND_HOST, () => {
    console.log(`[JS_RECEIVER] HTTP web listening on http://${BIND_HOST}:${HTTP_PORT}/`);
    console.log(`[JS_RECEIVER] WebSocket listening on ws://${BIND_HOST}:${HTTP_PORT}/`);
  });
}

startUdpReceiver();
startTcpReceiver();
startHttpServer();
