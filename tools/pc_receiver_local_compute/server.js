const dgram = require('node:dgram');
const http = require('node:http');
const net = require('node:net');
const fs = require('node:fs');
const path = require('node:path');

const MAGIC = 0x56535544;
const HEADER_SIZE = 20;
const PUBLIC_DIR = path.join(__dirname, 'public');

const BIND_HOST = process.env.BIND_HOST || '0.0.0.0';
const UDP_PORT = Number(process.env.UDP_PORT || 11000);
const TCP_PORT = Number(process.env.TCP_PORT || 11001);
const HTTP_PORT = Number(process.env.HTTP_PORT || 8180);

const latestByMode = {
  0: { jpeg: null, frameId: -1, updatedAtMs: 0, width: 0, height: 0, mode: 0 },
  1: { jpeg: null, frameId: -1, updatedAtMs: 0, width: 0, height: 0, mode: 1 },
  2: { jpeg: null, frameId: -1, updatedAtMs: 0, width: 0, height: 0, mode: 2 }
};
let latestStatus = { message: 'waiting' };
const inflightFrames = new Map();

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

function parseHeader(buf) {
  if (buf.length < HEADER_SIZE) return null;
  return {
    magic: buf.readUInt32BE(0),
    frameId: buf.readUInt32BE(4),
    chunkIdx: buf.readUInt16BE(8),
    chunkTotal: buf.readUInt16BE(10),
    payloadLen: buf.readUInt16BE(12),
    width: buf.readUInt16BE(14),
    height: buf.readUInt16BE(16),
    mode: buf.readUInt8(18)
  };
}

function isFrameNewer(prevFrameId, nextFrameId) {
  const diff = ((nextFrameId >>> 0) - (prevFrameId >>> 0)) >>> 0;
  return diff !== 0 && diff < 0x80000000;
}

function shouldAcceptFrame(mode, frameId, nowMs) {
  const latest = latestByMode[mode];
  if (!latest) return false;
  if (latest.frameId < 0) return true;
  if (isFrameNewer(latest.frameId, frameId)) return true;
  return (nowMs - latest.updatedAtMs) > 1500;
}

function cleanupInflight() {
  const nowMs = Date.now();
  for (const [frameId, entry] of inflightFrames.entries()) {
    if ((nowMs - entry.updatedAtMs) > 2000) {
      inflightFrames.delete(frameId);
    }
  }
}

function onUdpMessage(msg) {
  const hdr = parseHeader(msg);
  if (!hdr || hdr.magic !== MAGIC) return;
  if (!(hdr.mode in latestByMode)) return;
  if (hdr.chunkTotal === 0 || hdr.chunkIdx >= hdr.chunkTotal) return;
  if ((HEADER_SIZE + hdr.payloadLen) > msg.length) return;

  let entry = inflightFrames.get(hdr.frameId);
  if (!entry) {
    entry = {
      width: hdr.width,
      height: hdr.height,
      mode: hdr.mode,
      chunkTotal: hdr.chunkTotal,
      chunks: new Map(),
      updatedAtMs: Date.now()
    };
    inflightFrames.set(hdr.frameId, entry);
  }

  entry.chunks.set(hdr.chunkIdx, msg.subarray(HEADER_SIZE, HEADER_SIZE + hdr.payloadLen));
  entry.updatedAtMs = Date.now();

  if (entry.chunks.size !== entry.chunkTotal) return;

  const ordered = [];
  for (let i = 0; i < entry.chunkTotal; i += 1) {
    const chunk = entry.chunks.get(i);
    if (!chunk) return;
    ordered.push(chunk);
  }

  const jpeg = Buffer.concat(ordered);
  const nowMs = Date.now();
  if (shouldAcceptFrame(hdr.mode, hdr.frameId, nowMs)) {
    latestByMode[hdr.mode] = {
      jpeg,
      frameId: hdr.frameId >>> 0,
      updatedAtMs: nowMs,
      width: hdr.width,
      height: hdr.height,
      mode: hdr.mode
    };
  }
  inflightFrames.delete(hdr.frameId);
}

function startUdpReceiver() {
  const udp = dgram.createSocket('udp4');
  udp.on('message', onUdpMessage);
  udp.bind(UDP_PORT, BIND_HOST, () => {
    console.log(`[LOCAL_COMPUTE] UDP listening on ${BIND_HOST}:${UDP_PORT}`);
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
            // ignore malformed line
          }
        }
        idx = buffer.indexOf('\n');
      }
    });
  });

  server.listen(TCP_PORT, BIND_HOST, () => {
    console.log(`[LOCAL_COMPUTE] TCP listening on ${BIND_HOST}:${TCP_PORT}`);
  });
}

function writeJson(res, payload) {
  res.writeHead(200, {
    'Content-Type': 'application/json; charset=utf-8',
    'Cache-Control': 'no-store'
  });
  res.end(JSON.stringify(payload));
}

function writeJpeg(res, frame, notReadyMessage) {
  if (!frame || !frame.jpeg) {
    res.writeHead(503, {
      'Content-Type': 'text/plain; charset=utf-8',
      'Cache-Control': 'no-store'
    });
    res.end(notReadyMessage);
    return;
  }
  res.writeHead(200, {
    'Content-Type': 'image/jpeg',
    'Cache-Control': 'no-store'
  });
  res.end(frame.jpeg);
}

function startHttpServer() {
  const server = http.createServer((req, res) => {
    const reqUrl = new URL(req.url || '/', `http://${req.headers.host || 'localhost'}`);
    const pathname = reqUrl.pathname;

    if (pathname === '/' || pathname === '/index.html') {
      serveFile(res, path.join(PUBLIC_DIR, 'index.html'), 'text/html; charset=utf-8');
      return;
    }
    if (pathname === '/app.js') {
      serveFile(res, path.join(PUBLIC_DIR, 'app.js'), 'application/javascript; charset=utf-8');
      return;
    }
    if (pathname === '/pipeline_worker.js') {
      serveFile(res, path.join(PUBLIC_DIR, 'pipeline_worker.js'), 'application/javascript; charset=utf-8');
      return;
    }
    if (pathname === '/api/status') {
      writeJson(res, latestStatus);
      return;
    }
    if (pathname === '/api/frame_meta') {
      writeJson(res, {
        gray: latestByMode[1],
        binary: latestByMode[0],
        rgb: latestByMode[2]
      });
      return;
    }
    if (pathname === '/api/frame_gray.jpg') {
      writeJpeg(res, latestByMode[1], 'gray frame not ready');
      return;
    }
    if (pathname === '/api/frame_binary.jpg') {
      writeJpeg(res, latestByMode[0], 'binary frame not ready');
      return;
    }
    if (pathname === '/api/frame_rgb.jpg') {
      writeJpeg(res, latestByMode[2], 'rgb frame not ready');
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
    console.log(`[LOCAL_COMPUTE] HTTP listening on http://${BIND_HOST}:${HTTP_PORT}/`);
  });
}

startUdpReceiver();
startTcpReceiver();
startHttpServer();
