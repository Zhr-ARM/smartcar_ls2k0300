const dgram = require('node:dgram');
const http = require('node:http');
const fs = require('node:fs');
const net = require('node:net');
const path = require('node:path');

const MAGIC = 0x56535544; // VSUD
const HEADER_SIZE = 20;
const PUBLIC_DIR = path.join(__dirname, 'public');

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
  const server = http.createServer((req, res) => {
    if (req.url === '/' || req.url === '/index.html') {
      serveFile(res, path.join(PUBLIC_DIR, 'index.html'), 'text/html; charset=utf-8');
      return;
    }

    if (req.url === '/api/status') {
      const payload = JSON.stringify(latestStatus);
      res.writeHead(200, {
        'Content-Type': 'application/json; charset=utf-8',
        'Cache-Control': 'no-store'
      });
      res.end(payload);
      return;
    }

    if (req.url && req.url.startsWith('/api/frame_gray.jpg')) {
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

    if (req.url && req.url.startsWith('/api/frame_binary.jpg')) {
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

    if (req.url === '/healthz') {
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
