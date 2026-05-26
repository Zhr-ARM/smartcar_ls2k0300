const assert = require('node:assert/strict');
const fs = require('node:fs');
const http = require('node:http');
const path = require('node:path');
const { spawn } = require('node:child_process');

const ROOT = path.resolve(__dirname, '..');
const folder = `source_recording_test_${Date.now()}`;
const httpPort = 19390;
const imageB64 = Buffer.from('fake-jpeg-bytes').toString('base64');

function requestJson(method, pathname, body) {
  return new Promise((resolve, reject) => {
    const text = body ? JSON.stringify(body) : '';
    const req = http.request({
      host: '127.0.0.1',
      port: httpPort,
      method,
      path: pathname,
      headers: {
        'Content-Type': 'application/json',
        'Content-Length': Buffer.byteLength(text)
      }
    }, (res) => {
      const chunks = [];
      res.on('data', (chunk) => chunks.push(chunk));
      res.on('end', () => {
        const raw = Buffer.concat(chunks).toString('utf8');
        try {
          resolve({ statusCode: res.statusCode, json: raw ? JSON.parse(raw) : {} });
        } catch (err) {
          reject(err);
        }
      });
    });
    req.on('error', reject);
    if (text) req.write(text);
    req.end();
  });
}

function requestRaw(method, pathname) {
  return new Promise((resolve, reject) => {
    const req = http.request({
      host: '127.0.0.1',
      port: httpPort,
      method,
      path: pathname
    }, (res) => {
      const chunks = [];
      res.on('data', (chunk) => chunks.push(chunk));
      res.on('end', () => {
        resolve({
          statusCode: res.statusCode,
          headers: res.headers,
          body: Buffer.concat(chunks)
        });
      });
    });
    req.on('error', reject);
    req.end();
  });
}

async function waitForServer() {
  for (let i = 0; i < 40; i += 1) {
    try {
      const result = await requestJson('GET', '/api/status');
      if (result.statusCode === 200) return;
    } catch (_) {}
    await new Promise((resolve) => setTimeout(resolve, 100));
  }
  throw new Error('server did not start');
}

(async () => {
  const child = spawn(process.execPath, ['server.js'], {
    cwd: ROOT,
    env: {
      ...process.env,
      BIND_HOST: '127.0.0.1',
      UDP_PORT: '19300',
      TCP_PORT: '19301',
      HTTP_PORT: String(httpPort)
    },
    stdio: ['ignore', 'pipe', 'pipe']
  });

  try {
    await waitForServer();
    const save = await requestJson('POST', '/api/recordings/save', {
      folder,
      recorded_at_ms: 1000,
      duration_ms: 120,
      frame_count: 2,
      statuses: [
        { client_ts_ms: 1000, status: { route_main_state: 0, target_board_state: 1 } },
        { client_ts_ms: 1120, status: { route_main_state: 2, target_board_state: 3 } }
      ],
      source_frames: {
        gray: [
          { client_ts_ms: 1000, frame_id: 1, width: 160, height: 120, mime: 'image/jpeg', data_b64: imageB64 },
          { client_ts_ms: 1120, frame_id: 2, width: 160, height: 120, mime: 'image/jpeg', data_b64: imageB64 }
        ],
        binary: [
          { client_ts_ms: 1000, frame_id: 1, width: 160, height: 120, mime: 'image/jpeg', data_b64: imageB64 }
        ]
      },
      session_meta: { recording_kind: 'source_frames' }
    });
    assert.equal(save.statusCode, 200);
    assert.equal(save.json.ok, true);

    const load = await requestJson('GET', `/api/recordings/load?folder=${encodeURIComponent(folder)}`);
    assert.equal(load.statusCode, 200);
    assert.equal(load.json.ok, true);
    assert.equal(load.json.status.frame_count, 2);
    assert.equal(load.json.meta.source_frames.gray.length, 2);
    assert.match(load.json.source_frames.gray[0].url, /^\/api\/recordings\/file\?/);
    assert.equal(load.json.source_frames.gray[0].client_ts_ms, 1000);
    assert.equal(load.json.source_frames.binary[0].mode, 'binary');

    const frameFile = await requestRaw('GET', load.json.source_frames.gray[0].url);
    assert.equal(frameFile.statusCode, 200);
    assert.equal(frameFile.headers['content-type'], 'image/jpeg');
    assert.equal(frameFile.body.toString('utf8'), 'fake-jpeg-bytes');

    const traversal = await requestRaw(
      'GET',
      `/api/recordings/file?folder=${encodeURIComponent(folder)}&name=${encodeURIComponent('source_frames/gray/../000000.jpg')}`
    );
    assert.equal(traversal.statusCode, 400);

    console.log('[test_source_recording] PASS');
  } finally {
    child.kill('SIGTERM');
    fs.rmSync(path.join(ROOT, 'recordings', folder), { recursive: true, force: true });
  }
})().catch((err) => {
  console.error(err);
  process.exit(1);
});
