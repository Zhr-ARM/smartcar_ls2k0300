const dgram = require('node:dgram');
const http = require('node:http');
const fs = require('node:fs');
const path = require('node:path');
const crypto = require('node:crypto');
const { execFile } = require('node:child_process');
const WebSocket = require('ws');

const MAGIC = 0x56535545; // VSUE (unified protocol)
const HEADER_SIZE = 22;
const PUBLIC_DIR = path.join(__dirname, 'public');
const RECORDINGS_DIR = path.join(__dirname, 'recordings');
const ROOT_DIR = path.join(__dirname, '..', '..');
const SHARED_CONNECTION_PRESETS_PATH = path.join(ROOT_DIR, 'project', 'user', 'connection_presets.json');
const LOCAL_SMARTCAR_CONFIG_PATH = path.resolve(
  process.env.LOCAL_SMARTCAR_CONFIG_PATH || path.join(ROOT_DIR, 'project', 'user', 'smartcar_config.toml')
);

const BIND_HOST = process.env.BIND_HOST || '0.0.0.0';
const UDP_PORT = Number(process.env.UDP_PORT || 10000);
const HTTP_PORT = Number(process.env.HTTP_PORT || 8080);
const WEB_IMAGE_FORMAT_JPEG = 0;
const WEB_IMAGE_FORMAT_PNG = 1;
const WEB_IMAGE_FORMAT_BMP = 2;

const defaultBoardConnectionStore = (() => {
  const boardConfigBaseUrl = (process.env.BOARD_CONFIG_BASE_URL || 'http://172.21.79.138:18080').replace(/\/+$/, '');
  const boardConfigUrl = new URL(boardConfigBaseUrl);
  return {
    active_preset: 'hotspot_a',
    presets: [
      {
        id: 'hotspot_a',
        label: '热点 A',
        board_config_base_url: boardConfigBaseUrl,
        board_ssh_host: process.env.BOARD_SSH_HOST || boardConfigUrl.hostname,
        board_ssh_port: Number(process.env.BOARD_SSH_PORT || 22),
        board_ssh_user: process.env.BOARD_SSH_USER || 'root',
        board_ssh_target_path: process.env.BOARD_SSH_TARGET_PATH || '/home/root/tst/smartcar_config.toml',
        pc_receiver_ip: process.env.PC_RECEIVER_IP || '172.21.79.129',
        assistant_receiver_ip: process.env.ASSISTANT_RECEIVER_IP || (process.env.PC_RECEIVER_IP || '172.21.79.129'),
        build_target_host: process.env.BUILD_TARGET_HOST || (process.env.BOARD_SSH_HOST || boardConfigUrl.hostname),
        build_target_user: process.env.BUILD_TARGET_USER || (process.env.BOARD_SSH_USER || 'root'),
        build_target_port: Number(process.env.BUILD_TARGET_PORT || process.env.BOARD_SSH_PORT || 22),
        build_target_app_path: process.env.BUILD_TARGET_APP_PATH || '/home/root/tst',
        build_target_config_path: process.env.BUILD_TARGET_CONFIG_PATH || (process.env.BOARD_SSH_TARGET_PATH || '/home/root/tst/smartcar_config.toml')
      },
      {
        id: 'hotspot_b',
        label: '热点 B',
        board_config_base_url: 'http://10.119.73.138:18080',
        board_ssh_host: '10.119.73.138',
        board_ssh_port: 22,
        board_ssh_user: 'root',
        board_ssh_target_path: '/home/root/tst/smartcar_config.toml',
        pc_receiver_ip: '10.119.73.102',
        assistant_receiver_ip: '10.119.73.102',
        build_target_host: '10.119.73.138',
        build_target_user: 'root',
        build_target_port: 22,
        build_target_app_path: '/home/root/tst',
        build_target_config_path: '/home/root/tst/smartcar_config.toml'
      }
    ]
  };
})();

const latestFrame = {
  image: null, status: null, frameId: -1, updatedAtMs: 0, width: 0, height: 0, mode: 0, format: WEB_IMAGE_FORMAT_JPEG
};

const inflightFrames = new Map();
const udpByteEvents = [];
const udpFrameEvents = [];
let boardConnectionStore = loadBoardConnectionStore();

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

function broadcastWsBinary(buf) {
  if (!wss) return;
  for (const client of wss.clients) {
    if (client.readyState === WebSocket.OPEN) {
      client.send(buf);
    }
  }
}

function toBool01(value) {
  return value === true || value === 1 || value === '1';
}

function round3(value) {
  return Number.isFinite(value) ? Math.round(value * 1000) / 1000 : null;
}

function ensureRecordingsDir() {
  fs.mkdirSync(RECORDINGS_DIR, { recursive: true });
}

function normalizeBoardConnectionPreset(input, fallbackPreset = null) {
  const fallback = fallbackPreset || defaultBoardConnectionStore.presets[0];
  const rawBaseUrl = (input && input.board_config_base_url) || fallback.board_config_base_url;
  const boardConfigBaseUrl = String(rawBaseUrl || '').trim().replace(/\/+$/, '');
  let boardConfigUrl = null;
  try {
    boardConfigUrl = new URL(boardConfigBaseUrl);
  } catch (_) {
    throw new Error('board_config_base_url 无效，必须是 http://ip:port 形式');
  }
  const presetId = String((input && input.id) || fallback.id || '').trim();
  const presetLabel = String((input && input.label) || fallback.label || '').trim();
  const boardSshHost = String((input && input.board_ssh_host) || boardConfigUrl.hostname || '').trim();
  const boardSshPort = Number((input && input.board_ssh_port) || fallback.board_ssh_port || 22);
  const boardSshUser = String((input && input.board_ssh_user) || fallback.board_ssh_user || 'root').trim();
  const boardSshTargetPath = String((input && input.board_ssh_target_path) || fallback.board_ssh_target_path || '/home/root/tst/smartcar_config.toml').trim();
  const pcReceiverIp = String((input && input.pc_receiver_ip) || fallback.pc_receiver_ip || '').trim();
  const assistantReceiverIp = String((input && input.assistant_receiver_ip) || pcReceiverIp).trim();
  if (!presetId) throw new Error('preset id 不能为空');
  if (!/^[A-Za-z0-9._-]+$/.test(presetId)) throw new Error('preset id 只能包含字母、数字、点、下划线、短横线');
  if (!presetLabel) throw new Error('preset label 不能为空');
  if (!boardSshHost) throw new Error('board_ssh_host 不能为空');
  if (!Number.isFinite(boardSshPort) || boardSshPort <= 0) throw new Error('board_ssh_port 无效');
  if (!boardSshUser) throw new Error('board_ssh_user 不能为空');
  if (!boardSshTargetPath) throw new Error('board_ssh_target_path 不能为空');
  if (!pcReceiverIp) throw new Error('pc_receiver_ip 不能为空');
  if (!assistantReceiverIp) throw new Error('assistant_receiver_ip 不能为空');
  return {
    id: presetId,
    label: presetLabel,
    board_config_base_url: boardConfigBaseUrl,
    board_ssh_host: boardSshHost,
    board_ssh_port: Math.round(boardSshPort),
    board_ssh_user: boardSshUser,
    board_ssh_target_path: boardSshTargetPath,
    pc_receiver_ip: pcReceiverIp,
    assistant_receiver_ip: assistantReceiverIp,
    build_target_host: String((input && input.build_target_host) || boardSshHost).trim(),
    build_target_user: String((input && input.build_target_user) || boardSshUser).trim(),
    build_target_port: Math.round(Number((input && input.build_target_port) || boardSshPort || 22)),
    build_target_app_path: String((input && input.build_target_app_path) || '/home/root/tst').trim(),
    build_target_config_path: String((input && input.build_target_config_path) || boardSshTargetPath).trim()
  };
}

function normalizeBoardConnectionStore(input) {
  if (input && Array.isArray(input.presets)) {
    const normalizedPresets = input.presets.map((preset, index) =>
      normalizeBoardConnectionPreset(preset, defaultBoardConnectionStore.presets[index] || defaultBoardConnectionStore.presets[0]));
    const ids = new Set();
    for (const preset of normalizedPresets) {
      if (ids.has(preset.id)) throw new Error(`重复 preset id: ${preset.id}`);
      ids.add(preset.id);
    }
    const activePreset = String(input.active_preset || '').trim();
    if (!activePreset || !ids.has(activePreset)) throw new Error('active_preset 无效或不存在');
    return {
      active_preset: activePreset,
      presets: normalizedPresets
    };
  }

  const legacyPreset = normalizeBoardConnectionPreset(input || {}, defaultBoardConnectionStore.presets[0]);
  return {
    active_preset: legacyPreset.id || 'custom',
    presets: [legacyPreset]
  };
}

function loadBoardConnectionStore() {
  try {
    if (fs.existsSync(SHARED_CONNECTION_PRESETS_PATH)) {
      const text = fs.readFileSync(SHARED_CONNECTION_PRESETS_PATH, 'utf8');
      const parsed = text ? JSON.parse(text) : {};
      return normalizeBoardConnectionStore(parsed);
    }
  } catch (err) {
    console.warn(`[JS_RECEIVER] failed to load connection_presets.json: ${err.message}`);
  }
  return normalizeBoardConnectionStore(defaultBoardConnectionStore);
}

function saveBoardConnectionStore(store) {
  fs.writeFileSync(SHARED_CONNECTION_PRESETS_PATH, JSON.stringify(store, null, 2), 'utf8');
}

function getBoardConnectionStore() {
  return boardConnectionStore;
}

function getBoardConnectionSettings() {
  const store = getBoardConnectionStore();
  const active = store.presets.find((preset) => preset.id === store.active_preset);
  return active || store.presets[0];
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

function readTextBody(req, limitBytes = 2 * 1024 * 1024) {
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
      resolve(Buffer.concat(chunks).toString('utf8'));
    });
    req.on('error', reject);
  });
}

function proxyBoardConfig(pathname, method, bodyText = '') {
  return new Promise((resolve, reject) => {
    const settings = getBoardConnectionSettings();
    const upstream = new URL(`${settings.board_config_base_url}${pathname}`);
    const reqOptions = {
      hostname: upstream.hostname,
      port: upstream.port || 80,
      path: `${upstream.pathname}${upstream.search || ''}`,
      method,
      headers: {
        'Content-Type': 'text/plain; charset=utf-8',
        'Content-Length': Buffer.byteLength(bodyText, 'utf8')
      }
    };
    const upstreamReq = http.request(reqOptions, (upstreamRes) => {
      const chunks = [];
      upstreamRes.on('data', (chunk) => chunks.push(chunk));
      upstreamRes.on('end', () => {
        const text = Buffer.concat(chunks).toString('utf8');
        let json = {};
        try {
          json = text ? JSON.parse(text) : {};
        } catch (_) {
          json = { ok: false, message: text || `HTTP ${upstreamRes.statusCode}` };
        }
        if ((upstreamRes.statusCode || 500) >= 400) {
          reject(new Error(json && json.message ? json.message : `HTTP ${upstreamRes.statusCode}`));
          return;
        }
        resolve(json);
      });
    });
    upstreamReq.on('error', reject);
    if (bodyText) upstreamReq.write(bodyText);
    upstreamReq.end();
  });
}

function probeTcpPort(host, port, timeoutMs = 1200) {
  return new Promise((resolve) => {
    const socket = new net.Socket();
    let settled = false;
    const finish = (online, error = '') => {
      if (settled) return;
      settled = true;
      socket.destroy();
      resolve({ online, error });
    };
    socket.setTimeout(timeoutMs);
    socket.once('connect', () => finish(true, ''));
    socket.once('timeout', () => finish(false, 'connect timeout'));
    socket.once('error', (err) => finish(false, String(err && err.message ? err.message : err)));
    socket.connect(port, host);
  });
}

function probeBoardRuntimeHttp() {
  return new Promise((resolve) => {
    const settings = getBoardConnectionSettings();
    const boardConfigUrl = new URL(settings.board_config_base_url);
    const req = http.request({
      hostname: boardConfigUrl.hostname,
      port: boardConfigUrl.port || 80,
      path: '/healthz',
      method: 'GET',
      timeout: 1500
    }, (res) => {
      const chunks = [];
      res.on('data', (chunk) => chunks.push(chunk));
      res.on('end', () => {
        const text = Buffer.concat(chunks).toString('utf8').trim();
        if (res.statusCode === 200 && text === 'ok') {
          resolve({ online: true, error: '' });
          return;
        }
        resolve({ online: false, error: text || `HTTP ${res.statusCode}` });
      });
    });
    req.on('timeout', () => {
      req.destroy(new Error('connect timeout'));
    });
    req.on('error', (err) => {
      resolve({ online: false, error: String(err && err.message ? err.message : err) });
    });
    req.end();
  });
}

function buildConfigStatusPayload() {
  const settings = getBoardConnectionSettings();
  return Promise.all([
    probeBoardRuntimeHttp(),
    probeTcpPort(settings.board_ssh_host, settings.board_ssh_port)
  ]).then(([runtimeHttp, sshPort]) => ({
    ok: true,
    web_proxy_origin: `http://${BIND_HOST}:${HTTP_PORT}`,
    local_config_path: LOCAL_SMARTCAR_CONFIG_PATH,
    connection_store: getBoardConnectionStore(),
    connection_settings: settings,
    board_config_base_url: settings.board_config_base_url,
    runtime_http: runtimeHttp,
    ssh_transport: {
      host: settings.board_ssh_host,
      port: settings.board_ssh_port,
      user: settings.board_ssh_user,
      target_path: settings.board_ssh_target_path,
      online: sshPort.online,
      error: sshPort.error || ''
    }
  }));
}

function execFileAsync(file, args) {
  return new Promise((resolve, reject) => {
    execFile(file, args, { timeout: 10000, maxBuffer: 1024 * 1024 }, (error, stdout, stderr) => {
      if (error) {
        const detail = [stderr, stdout, error.message].filter(Boolean).join('\n').trim();
        reject(new Error(detail || `${file} failed`));
        return;
      }
      resolve({ stdout, stderr });
    });
  });
}

function persistLocalConfigToml(tomlText) {
  const targetPath = LOCAL_SMARTCAR_CONFIG_PATH;
  const parentDir = path.dirname(targetPath);
  fs.mkdirSync(parentDir, { recursive: true });
  const tmpPath = `${targetPath}.tmp.${process.pid}.${Date.now()}`;
  try {
    fs.writeFileSync(tmpPath, tomlText, 'utf8');
    fs.renameSync(tmpPath, targetPath);
  } catch (err) {
    try {
      fs.unlinkSync(tmpPath);
    } catch (_) {
      // ignore cleanup failure
    }
    throw err;
  }
  return targetPath;
}

function persistLocalConfigTomlSafely(tomlText) {
  try {
    const pathSaved = persistLocalConfigToml(tomlText);
    return {
      ok: true,
      path: pathSaved,
      message: 'local config updated'
    };
  } catch (err) {
    return {
      ok: false,
      path: LOCAL_SMARTCAR_CONFIG_PATH,
      message: String(err && err.message ? err.message : err)
    };
  }
}

function readLocalConfigTomlText() {
  return fs.readFileSync(LOCAL_SMARTCAR_CONFIG_PATH, 'utf8');
}

function normalizeTomlForComparison(text) {
  return String(text || '')
    .replace(/\r\n/g, '\n')
    .replace(/\r/g, '\n')
    .replace(/[ \t]+$/gm, '')
    .replace(/\n+$/g, '');
}

function buildTomlVerifyResult(expectedText, actualText) {
  const expected = String(expectedText || '');
  const actual = String(actualText || '');
  const sameExact = expected === actual;
  const sameNormalized = normalizeTomlForComparison(expected) === normalizeTomlForComparison(actual);
  return {
    same: sameExact || sameNormalized,
    same_exact: sameExact,
    same_normalized: sameNormalized,
    expected_bytes: Buffer.byteLength(expected, 'utf8'),
    actual_bytes: Buffer.byteLength(actual, 'utf8')
  };
}

function sha256Hex(text) {
  return crypto.createHash('sha256').update(String(text || ''), 'utf8').digest('hex');
}

function buildTriPartyVerify(webText, boardText, localText) {
  const web = String(webText || '');
  const board = String(boardText || '');
  const local = String(localText || '');
  const webBoard = buildTomlVerifyResult(web, board);
  const boardLocal = buildTomlVerifyResult(board, local);
  const webLocal = buildTomlVerifyResult(web, local);
  return {
    ok: true,
    all_same: !!(webBoard.same && boardLocal.same && webLocal.same),
    web_board_same: !!webBoard.same,
    board_local_same: !!boardLocal.same,
    web_local_same: !!webLocal.same,
    web_hash_sha256: sha256Hex(web),
    board_hash_sha256: sha256Hex(board),
    local_hash_sha256: sha256Hex(local),
    web_bytes: Buffer.byteLength(web, 'utf8'),
    board_bytes: Buffer.byteLength(board, 'utf8'),
    local_bytes: Buffer.byteLength(local, 'utf8'),
    web_board_verify: webBoard,
    board_local_verify: boardLocal,
    web_local_verify: webLocal
  };
}

async function pushConfigViaScp(tomlText) {
  const settings = getBoardConnectionSettings();
  const tmpPath = path.join(os.tmpdir(), `smartcar_config_${Date.now()}_${process.pid}.toml`);
  fs.writeFileSync(tmpPath, tomlText, 'utf8');
  try {
    const remote = `${settings.board_ssh_user}@${settings.board_ssh_host}:${settings.board_ssh_target_path}`;
    await execFileAsync('scp', [
      '-O',
      '-P', String(settings.board_ssh_port),
      '-o', 'BatchMode=yes',
      '-o', 'ConnectTimeout=5',
      '-o', 'StrictHostKeyChecking=accept-new',
      tmpPath,
      remote
    ]);
  } finally {
    try {
      fs.unlinkSync(tmpPath);
    } catch (_) {
      // ignore cleanup failure
    }
  }
}

async function pullConfigViaSsh() {
  const settings = getBoardConnectionSettings();
  const remote = `${settings.board_ssh_user}@${settings.board_ssh_host}`;
  const result = await execFileAsync('ssh', [
    '-p', String(settings.board_ssh_port),
    '-o', 'BatchMode=yes',
    '-o', 'ConnectTimeout=5',
    '-o', 'StrictHostKeyChecking=accept-new',
    remote,
    `cat '${settings.board_ssh_target_path.replace(/'/g, `'\\''`)}'`
  ]);
  return result.stdout;
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
    statuses: Array.isArray(payload.statuses) ? payload.statuses : [],
    session_meta: (payload.session_meta && typeof payload.session_meta === 'object') ? payload.session_meta : {}
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
    session_meta: statusPayload.session_meta,
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
  const statusLen = buf.readUInt16BE(20);
  return { magic, frameId, chunkIdx, chunkTotal, payloadLen, width, height, mode, format, statusLen };
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
  if (prev < 1024) return false;
  return next < 64;
}

function shouldAcceptFrame(frameId, nowMs) {
  if (latestFrame.frameId < 0) return true;
  if (isFrameNewer(latestFrame.frameId, frameId)) return true;
  if (isLikelyFrameCounterReset(latestFrame.frameId, frameId)) return true;
  if ((nowMs - latestFrame.updatedAtMs) > 1500) return true;
  return false;
}

function onUdpMessage(msg) {
  udpByteEvents.push({ ts: Date.now(), bytes: msg.length });
  const hdr = parseHeader(msg);
  if (!hdr) return;
  if (hdr.magic !== MAGIC) return;
  if (hdr.mode !== 0 && hdr.mode !== 1) return;
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
      statusLen: (hdr.chunkIdx === 0) ? hdr.statusLen : 0,
      ts: Date.now()
    };
    inflightFrames.set(hdr.frameId, entry);
  }

  // chunk 0 carries the statusLen; store it
  if (hdr.chunkIdx === 0 && hdr.statusLen > 0) {
    entry.statusLen = hdr.statusLen;
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
    const combined = Buffer.concat(ordered);
    const nowMs = Date.now();

    if (shouldAcceptFrame(hdr.frameId, nowMs)) {
      // Split: first statusLen bytes = JSON status, remainder = JPEG image
      const statusLen = entry.statusLen || 0;
      let status = null;
      let image = combined;
      if (statusLen > 0 && statusLen < combined.length) {
        const statusBuf = combined.subarray(0, statusLen);
        image = combined.subarray(statusLen);
        try {
          status = JSON.parse(statusBuf.toString('utf8'));
        } catch (_) {
          status = null;
        }
      }

      const wireBytes = ordered.reduce((sum, chunk) => sum + chunk.length, 0) + (entry.chunkTotal * HEADER_SIZE);

      latestFrame.image = image;
      latestFrame.status = status;
      latestFrame.frameId = hdr.frameId >>> 0;
      latestFrame.updatedAtMs = nowMs;
      latestFrame.width = hdr.width;
      latestFrame.height = hdr.height;
      latestFrame.mode = hdr.mode;
      latestFrame.format = sanitizeImageFormat(hdr.format);

      // Build binary WebSocket message: [status_len: uint16 BE] [status_json] [image_jpeg]
      if (status) {
        const statusJson = Buffer.from(JSON.stringify(status), 'utf8');
        const header = Buffer.allocUnsafe(2);
        header.writeUInt16BE(statusJson.length, 0);
        broadcastWsBinary(Buffer.concat([header, statusJson, image]));
      }

      udpFrameEvents.push({ ts: nowMs, wireBytes });
    }
    inflightFrames.delete(hdr.frameId);
  }
}

function buildTransportTelemetry() {
  const now = Date.now();
  const oneSecAgo = now - 1000;
  const recentBytes = udpByteEvents.filter((item) => item.ts >= oneSecAgo);
  const recentFrames = udpFrameEvents.filter((item) => item.ts >= oneSecAgo);

  const rxBytesPerSec = recentBytes.reduce((sum, item) => sum + item.bytes, 0);
  const rxFramesPerSec = recentFrames.length;
  const totalWireBytes = recentFrames.reduce((sum, item) => sum + item.wireBytes, 0);
  const avgWireBytes = rxFramesPerSec > 0 ? Math.round(totalWireBytes / rxFramesPerSec) : null;

  return {
    rx_udp_bytes_per_sec: rxBytesPerSec,
    rx_udp_kib_per_sec: round3(rxBytesPerSec / 1024),
    rx_udp_mbps: round3((rxBytesPerSec * 8) / 1000000),
    rx_udp_frames_per_sec: rxFramesPerSec,
    rx_udp_avg_frame_bytes: avgWireBytes
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
    if (pathname === '/config.html') {
      serveFile(res, path.join(PUBLIC_DIR, 'config.html'), 'text/html; charset=utf-8');
      return;
    }
    if (pathname === '/playback.html') {
      serveFile(res, path.join(PUBLIC_DIR, 'playback.html'), 'text/html; charset=utf-8');
      return;
    }
    if (pathname === '/playback_app.js') {
      serveFile(res, path.join(PUBLIC_DIR, 'playback_app.js'), 'application/javascript; charset=utf-8');
      return;
    }
    if (pathname === '/config_app.js') {
      serveFile(res, path.join(PUBLIC_DIR, 'config_app.js'), 'application/javascript; charset=utf-8');
      return;
    }
    if (pathname === '/shared_receiver_core.js') {
      serveFile(res, path.join(PUBLIC_DIR, 'shared_receiver_core.js'), 'application/javascript; charset=utf-8');
      return;
    }

    if (pathname === '/api/status') {
      const payload = JSON.stringify(Object.assign({}, latestFrame.status || {}, buildTransportTelemetry()));
      res.writeHead(200, {
        'Content-Type': 'application/json; charset=utf-8',
        'Cache-Control': 'no-store'
      });
      res.end(payload);
      return;
    }

    if (pathname === '/api/config/current') {
      proxyBoardConfig('/api/config/current', 'GET')
        .then((result) => {
          sendJson(res, 200, Object.assign({ ok: true, board_config_base_url: getBoardConnectionSettings().board_config_base_url }, result));
        })
        .catch((err) => {
          sendJson(res, 502, {
            ok: false,
            message: String(err && err.message ? err.message : err),
            board_config_base_url: getBoardConnectionSettings().board_config_base_url
          });
        });
      return;
    }

    if (pathname === '/api/config/connection_settings') {
      if (req.method === 'GET') {
        sendJson(res, 200, {
          ok: true,
          connection_store: getBoardConnectionStore(),
          connection_settings: getBoardConnectionSettings()
        });
        return;
      }
      if (req.method === 'POST') {
        readJsonBody(req, 128 * 1024).then((payload) => {
          const normalized = normalizeBoardConnectionStore(payload || {});
          boardConnectionStore = normalized;
          saveBoardConnectionStore(normalized);
          return buildConfigStatusPayload();
        }).then((status) => {
          sendJson(res, 200, status);
        }).catch((err) => {
          sendJson(res, 400, { ok: false, message: String(err && err.message ? err.message : err) });
        });
        return;
      }
    }

    if (pathname === '/api/config/status') {
      buildConfigStatusPayload()
        .then((result) => sendJson(res, 200, result))
        .catch((err) => sendJson(res, 500, { ok: false, message: String(err && err.message ? err.message : err) }));
      return;
    }

    if (pathname === '/api/config/apply' && req.method === 'POST') {
      readTextBody(req).then(async (bodyText) => {
        const applyResult = await proxyBoardConfig('/api/config/apply', 'POST', bodyText);

        let boardVerify = {
          ok: false,
          same: false,
          same_exact: false,
          same_normalized: false,
          expected_bytes: Buffer.byteLength(String(bodyText || ''), 'utf8'),
          actual_bytes: 0,
          loaded_path: '',
          message: ''
        };
        try {
          const boardCurrent = await proxyBoardConfig('/api/config/current', 'GET');
          boardVerify = Object.assign({
            ok: true,
            loaded_path: boardCurrent.loaded_path || '',
            message: 'board config verified'
          }, buildTomlVerifyResult(bodyText, String(boardCurrent.toml_text || '')));
        } catch (err) {
          boardVerify = Object.assign(boardVerify, {
            message: String(err && err.message ? err.message : err)
          });
        }
        return { result: applyResult, boardVerify };
      }).then(({ result, boardVerify }) => {
        sendJson(res, 200, Object.assign({
          ok: true,
          board_config_base_url: getBoardConnectionSettings().board_config_base_url,
          local_config_path: LOCAL_SMARTCAR_CONFIG_PATH,
          board_verify: boardVerify
        }, result));
      }).catch((err) => {
        sendJson(res, 502, {
          ok: false,
          message: String(err && err.message ? err.message : err),
          board_config_base_url: getBoardConnectionSettings().board_config_base_url
        });
      });
      return;
    }

    if (pathname === '/api/config/ssh_push' && req.method === 'POST') {
      readTextBody(req).then(async (bodyText) => {
        await pushConfigViaScp(bodyText);
        const settings = getBoardConnectionSettings();
        const boardText = await pullConfigViaSsh();
        const boardVerify = Object.assign({
          ok: true,
          message: 'board file verified via ssh'
        }, buildTomlVerifyResult(bodyText, boardText));

        sendJson(res, 200, {
          ok: true,
          message: 'ssh uploaded',
          board_config_base_url: settings.board_config_base_url,
          local_config_path: LOCAL_SMARTCAR_CONFIG_PATH,
          board_verify: boardVerify,
          ssh_target: {
            host: settings.board_ssh_host,
            port: settings.board_ssh_port,
            user: settings.board_ssh_user,
            target_path: settings.board_ssh_target_path
          }
        });
      }).catch((err) => {
        const settings = getBoardConnectionSettings();
        sendJson(res, 502, {
          ok: false,
          message: String(err && err.message ? err.message : err),
          ssh_target: {
            host: settings.board_ssh_host,
            port: settings.board_ssh_port,
            user: settings.board_ssh_user,
            target_path: settings.board_ssh_target_path
          }
        });
      });
      return;
    }

    if (pathname === '/api/config/local_update' && req.method === 'POST') {
      readTextBody(req).then((bodyText) => {
        const localSync = persistLocalConfigTomlSafely(bodyText);
        let localVerify = {
          ok: false,
          same: false,
          same_exact: false,
          same_normalized: false,
          expected_bytes: Buffer.byteLength(String(bodyText || ''), 'utf8'),
          actual_bytes: 0,
          path: LOCAL_SMARTCAR_CONFIG_PATH,
          message: ''
        };
        if (localSync.ok) {
          try {
            const localText = readLocalConfigTomlText();
            localVerify = Object.assign({
              ok: true,
              path: LOCAL_SMARTCAR_CONFIG_PATH,
              message: 'local config verified'
            }, buildTomlVerifyResult(bodyText, localText));
          } catch (err) {
            localVerify = Object.assign(localVerify, {
              message: String(err && err.message ? err.message : err)
            });
          }
        } else {
          localVerify = Object.assign(localVerify, {
            message: `local sync failed: ${localSync.message || 'unknown error'}`
          });
        }
        sendJson(res, localSync.ok ? 200 : 500, {
          ok: !!localSync.ok,
          message: localSync.ok ? 'local config updated' : 'local config update failed',
          local_config_path: LOCAL_SMARTCAR_CONFIG_PATH,
          local_sync: localSync,
          local_verify: localVerify
        });
      }).catch((err) => {
        sendJson(res, 500, {
          ok: false,
          message: String(err && err.message ? err.message : err),
          local_config_path: LOCAL_SMARTCAR_CONFIG_PATH
        });
      });
      return;
    }

    if (pathname === '/api/config/ssh_pull') {
      pullConfigViaSsh().then((tomlText) => {
        const settings = getBoardConnectionSettings();
        sendJson(res, 200, {
          ok: true,
          message: 'ssh downloaded',
          toml_text: tomlText,
          board_config_base_url: settings.board_config_base_url,
          ssh_target: {
            host: settings.board_ssh_host,
            port: settings.board_ssh_port,
            user: settings.board_ssh_user,
            target_path: settings.board_ssh_target_path
          }
        });
      }).catch((err) => {
        const settings = getBoardConnectionSettings();
        sendJson(res, 502, {
          ok: false,
          message: String(err && err.message ? err.message : err),
          ssh_target: {
            host: settings.board_ssh_host,
            port: settings.board_ssh_port,
            user: settings.board_ssh_user,
            target_path: settings.board_ssh_target_path
          }
        });
      });
      return;
    }

    if (pathname === '/api/frame_meta') {
      sendJson(res, 200, {
        frameId: latestFrame.frameId,
        width: latestFrame.width,
        height: latestFrame.height,
        mode: latestFrame.mode,
        format: latestFrame.format,
        updatedAtMs: latestFrame.updatedAtMs
      });
      return;
    }

    if (pathname === '/api/frame.jpg') {
      writeImage(res, latestFrame, 'frame not ready');
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
    if (latestFrame.status) {
      ws.send(JSON.stringify({ type: 'status', data: latestFrame.status, ts: Date.now() }));
    }
  });

  server.listen(HTTP_PORT, BIND_HOST, () => {
    console.log(`[JS_RECEIVER] HTTP web listening on http://${BIND_HOST}:${HTTP_PORT}/`);
    console.log(`[JS_RECEIVER] WebSocket listening on ws://${BIND_HOST}:${HTTP_PORT}/`);
  });
}

startUdpReceiver();
startHttpServer();
