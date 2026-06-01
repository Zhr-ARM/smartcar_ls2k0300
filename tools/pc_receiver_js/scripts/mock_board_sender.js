#!/usr/bin/env node
'use strict';

const dgram = require('node:dgram');
const fs = require('node:fs');
const net = require('node:net');
const path = require('node:path');

const MAGIC = 0x56535544; // VSUD
const HEADER_SIZE = 20;
const FORMAT_JPEG = 0;
const FORMAT_PNG = 1;
const FORMAT_BMP = 2;
const MODE_BINARY = 0;
const MODE_GRAY = 1;
const MODE_RGB = 2;
const MODE_ROI64 = 3;
const MODE_BY_NAME = { binary: MODE_BINARY, gray: MODE_GRAY, rgb: MODE_RGB, roi64: MODE_ROI64 };
const DEFAULT_HOST = '127.0.0.1';
const DEFAULT_UDP_PORT = 10000;
const DEFAULT_TCP_PORT = 10001;
const DEFAULT_FPS = 10;
const DEFAULT_CHUNK_BYTES = 1200;

function usage() {
  console.log(`Usage:
  node scripts/mock_board_sender.js [options]

Options:
  --host <ip>         Receiver host, default ${DEFAULT_HOST}
  --udp-port <port>   Receiver UDP image port, default ${DEFAULT_UDP_PORT}
  --tcp-port <port>   Receiver TCP status port, default ${DEFAULT_TCP_PORT}
  --fixture <dir>     Replay a fixture directory recorded by record_live_fixture.js
  --duration <sec>    Stop after N seconds, default 0 (run until Ctrl+C)
  --fps <n>           Send rate when synthetic or fixture has no timing, default ${DEFAULT_FPS}
  --modes <list>      Comma-separated image modes, default gray,binary,rgb,roi64
  --status-only       Send TCP status only
  --once              Send one sample and exit
  --help              Show this help
`);
}

function parseArgs(argv) {
  const opts = {
    host: DEFAULT_HOST,
    udpPort: DEFAULT_UDP_PORT,
    tcpPort: DEFAULT_TCP_PORT,
    fixtureDir: '',
    durationSec: 0,
    fps: DEFAULT_FPS,
    modes: ['gray', 'binary', 'rgb', 'roi64'],
    statusOnly: false,
    once: false
  };

  for (let i = 0; i < argv.length; i += 1) {
    const arg = argv[i];
    if (arg === '--help' || arg === '-h') {
      usage();
      process.exit(0);
    }
    if (arg === '--status-only') {
      opts.statusOnly = true;
      continue;
    }
    if (arg === '--once') {
      opts.once = true;
      continue;
    }
    const next = argv[i + 1];
    if (arg === '--host') {
      opts.host = String(next || '').trim();
      i += 1;
    } else if (arg === '--udp-port') {
      opts.udpPort = Number(next);
      i += 1;
    } else if (arg === '--tcp-port') {
      opts.tcpPort = Number(next);
      i += 1;
    } else if (arg === '--fixture') {
      opts.fixtureDir = path.resolve(String(next || ''));
      i += 1;
    } else if (arg === '--duration') {
      opts.durationSec = Number(next);
      i += 1;
    } else if (arg === '--fps') {
      opts.fps = Number(next);
      i += 1;
    } else if (arg === '--modes') {
      opts.modes = String(next || '').split(',').map((item) => item.trim()).filter(Boolean);
      i += 1;
    } else {
      throw new Error(`Unknown option: ${arg}`);
    }
  }

  if (!opts.host) throw new Error('--host cannot be empty');
  if (!Number.isInteger(opts.udpPort) || opts.udpPort <= 0) throw new Error('--udp-port must be a positive integer');
  if (!Number.isInteger(opts.tcpPort) || opts.tcpPort <= 0) throw new Error('--tcp-port must be a positive integer');
  if (!Number.isFinite(opts.durationSec) || opts.durationSec < 0) throw new Error('--duration must be >= 0');
  if (!Number.isFinite(opts.fps) || opts.fps <= 0) throw new Error('--fps must be > 0');
  const invalidModes = opts.modes.filter((mode) => !Object.prototype.hasOwnProperty.call(MODE_BY_NAME, mode));
  if (invalidModes.length > 0) throw new Error(`Unknown mode(s): ${invalidModes.join(', ')}`);
  return opts;
}

function sleep(ms) {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

function readJson(filePath) {
  return JSON.parse(fs.readFileSync(filePath, 'utf8'));
}

function inferFormat(filePath) {
  const ext = path.extname(filePath).toLowerCase();
  if (ext === '.png') return FORMAT_PNG;
  if (ext === '.bmp') return FORMAT_BMP;
  return FORMAT_JPEG;
}

function readImageSize(buffer, format) {
  if (format === FORMAT_PNG && buffer.length >= 24 && buffer.toString('ascii', 1, 4) === 'PNG') {
    return { width: buffer.readUInt32BE(16), height: buffer.readUInt32BE(20) };
  }
  if (format === FORMAT_BMP && buffer.length >= 26 && buffer.toString('ascii', 0, 2) === 'BM') {
    return { width: buffer.readInt32LE(18), height: Math.abs(buffer.readInt32LE(22)) };
  }
  if (format === FORMAT_JPEG) {
    let offset = 2;
    while (offset + 8 < buffer.length) {
      if (buffer[offset] !== 0xff) {
        offset += 1;
        continue;
      }
      const marker = buffer[offset + 1];
      const size = buffer.readUInt16BE(offset + 2);
      if (marker >= 0xc0 && marker <= 0xcf && marker !== 0xc4 && marker !== 0xc8 && marker !== 0xcc) {
        return { width: buffer.readUInt16BE(offset + 7), height: buffer.readUInt16BE(offset + 5) };
      }
      offset += 2 + size;
    }
  }
  return { width: 160, height: 60 };
}

function makeBmp(width, height, pixelAt) {
  const rowBytes = Math.ceil((width * 3) / 4) * 4;
  const pixelBytes = rowBytes * height;
  const fileBytes = 54 + pixelBytes;
  const buffer = Buffer.alloc(fileBytes);
  buffer.write('BM', 0, 'ascii');
  buffer.writeUInt32LE(fileBytes, 2);
  buffer.writeUInt32LE(54, 10);
  buffer.writeUInt32LE(40, 14);
  buffer.writeInt32LE(width, 18);
  buffer.writeInt32LE(height, 22);
  buffer.writeUInt16LE(1, 26);
  buffer.writeUInt16LE(24, 28);
  buffer.writeUInt32LE(pixelBytes, 34);

  for (let y = 0; y < height; y += 1) {
    const outY = height - 1 - y;
    for (let x = 0; x < width; x += 1) {
      const [r, g, b] = pixelAt(x, y);
      const idx = 54 + outY * rowBytes + x * 3;
      buffer[idx] = b;
      buffer[idx + 1] = g;
      buffer[idx + 2] = r;
    }
  }
  return buffer;
}

function clampByte(value) {
  return Math.max(0, Math.min(255, Math.round(value)));
}

function syntheticImage(modeName, seq) {
  const width = modeName === 'roi64' ? 64 : 160;
  const height = modeName === 'roi64' ? 64 : 60;
  const phase = seq % 80;
  const image = makeBmp(width, height, (x, y) => {
    const laneCenter = width / 2 + Math.sin((seq + y) / 12) * 18;
    const lane = Math.abs(x - laneCenter) < 2 || Math.abs(x - laneCenter + 38) < 2 || Math.abs(x - laneCenter - 38) < 2;
    if (modeName === 'binary') return lane || y === phase % height ? [255, 255, 255] : [0, 0, 0];
    if (modeName === 'rgb' || modeName === 'roi64') {
      return [
        clampByte(30 + x * 1.2),
        clampByte(50 + y * 2.4),
        lane ? 245 : clampByte(150 + Math.sin((x + seq) / 9) * 70)
      ];
    }
    const v = lane ? 245 : clampByte(35 + (x / width) * 120 + Math.sin((y + seq) / 7) * 35);
    return [v, v, v];
  });
  return { image, width, height, format: FORMAT_BMP };
}

function makeSyntheticStatus(seq) {
  const lineError = Math.round(Math.sin(seq / 8) * 28);
  const speed = 150 + Math.round(Math.sin(seq / 18) * 18);
  const mainState = Math.floor(seq / 80) % 3;
  return {
    web_data_profile: 0,
    web_full_debug: 1,
    udp_web_max_fps: 10,
    capture_thread_fps: 150 + (seq % 7),
    vision_process_fps: 148 + (seq % 9),
    udp_tx_fps: 10,
    udp_web_send_gray: 1,
    udp_web_send_binary: 1,
    udp_web_send_rgb: 1,
    udp_web_send_roi64: 1,
    udp_web_gray_image_format: FORMAT_BMP,
    udp_web_binary_image_format: FORMAT_BMP,
    udp_web_rgb_image_format: FORMAT_BMP,
    line_error: lineError,
    zebra_cross_count: Math.floor(seq / 120) % 3,
    cpu_usage_percent: 35 + (seq % 20),
    mem_usage_percent: 18.4,
    base_speed: 150,
    adjusted_base_speed: speed,
    left_target_count: speed - lineError,
    right_target_count: speed + lineError,
    left_current_count: 80 + Math.round(Math.sin(seq / 5) * 4),
    right_current_count: 82 + Math.round(Math.cos(seq / 6) * 4),
    otsu_threshold: 120 + (seq % 12),
    total_us: 6200 + (seq % 500),
    maze_left_points_raw: 50,
    maze_right_points_raw: 50,
    infer_enabled: 0,
    ncnn_enabled: 0,
    ncnn_has_result: 0,
    ncnn_infer_valid: 0,
    ncnn_infer_us: 0,
    ncnn_top_class_id: -1,
    ncnn_top_score: 0,
    ncnn_top_label: '',
    ncnn_probs: [],
    pid_common_route_main_state: mainState,
    pid_common_route_sub_state: Math.floor(seq / 25) % 6,
    route_main_state: mainState,
    route_sub_state: Math.floor(seq / 25) % 6,
    pid_common_applied_base_speed: speed,
    pid_common_raw_error_px: lineError,
    pid_common_filtered_error_px: lineError * 0.8,
    pid_common_control_error_px: lineError * 0.6,
    pid_common_track_point_valid: 1,
    pid_common_track_point: [80 + lineError, 58],
    pid_common_position_pid_target: 0,
    pid_common_position_pid_error: lineError * 0.6,
    pid_common_position_pid_output: Math.round(lineError * 0.6 * 3.0),
    pid_common_yaw_rate_ref_dps: Math.round(Math.sin(seq / 10) * 120),
    pid_common_measured_yaw_rate_dps: Math.round(Math.sin(seq / 10) * 120 + (Math.random() - 0.5) * 20),
    pid_common_yaw_rate_error_dps: Math.round((Math.random() - 0.5) * 20),
    pid_common_yaw_pid_target: Math.round(Math.sin(seq / 10) * 120),
    pid_common_yaw_pid_error: Math.round((Math.random() - 0.5) * 20),
    pid_common_yaw_pid_output: Math.round(Math.sin(seq / 10) * 30),
    pid_common_speed_command_base: speed,
    pid_common_speed_command_diff: 0,
    pid_left_target_count: speed - lineError,
    pid_right_target_count: speed + lineError,
    pid_left_current_count: 80 + Math.round(Math.sin(seq / 5) * 4),
    pid_right_current_count: 82 + Math.round(Math.cos(seq / 6) * 4),
    pid_left_feedback: 80 + Math.round(Math.sin(seq / 5) * 3),
    pid_right_feedback: 82 + Math.round(Math.cos(seq / 6) * 3),
    pid_left_filtered_count: 80 + Math.round(Math.sin(seq / 5) * 3),
    pid_right_filtered_count: 82 + Math.round(Math.cos(seq / 6) * 3),
    gray_size: [160, 60],
    ipm_size: [160, 100],
    left_boundary: makeBoundary(-34, seq),
    right_boundary: makeBoundary(34, seq),
    ipm_left_boundary: makeBoundary(-40, seq),
    ipm_right_boundary: makeBoundary(40, seq),
    ipm_centerline_selected_count: 21,
    src_centerline_selected_count: 21,
    mock_board: 1,
    mock_seq: seq,
    ts_ms: Date.now()
  };
}

function makeBoundary(offset, seq) {
  const points = [];
  for (let y = 0; y < 60; y += 3) {
    const x = Math.round(80 + offset + Math.sin((seq + y) / 16) * 7);
    points.push([x, y]);
  }
  return points;
}

function loadFixture(fixtureDir, modes) {
  if (!fixtureDir) return null;
  const statusPath = path.join(fixtureDir, 'status.json');
  const payload = readJson(statusPath);
  const statuses = Array.isArray(payload.statuses) ? payload.statuses : [];
  const samples = Array.isArray(payload.samples) ? payload.samples : [];
  if (statuses.length < 1 && samples.length < 1) {
    throw new Error(`Fixture has no statuses or samples: ${statusPath}`);
  }

  return statuses.map((entry, index) => {
    const sample = samples[index] || {};
    const status = entry && entry.status ? entry.status : (sample.status || {});
    const frames = {};
    for (const modeName of modes) {
      const frameInfo = sample.frames && sample.frames[modeName];
      if (!frameInfo || !frameInfo.file) continue;
      const filePath = path.join(fixtureDir, frameInfo.file);
      if (!fs.existsSync(filePath)) continue;
      const image = fs.readFileSync(filePath);
      const format = inferFormat(filePath);
      const size = readImageSize(image, format);
      frames[modeName] = { image, width: size.width, height: size.height, format };
    }
    return {
      tMs: Number(entry && entry.t_ms) || Number(sample.t_ms) || 0,
      status,
      frames
    };
  });
}

function connectTcp(host, port) {
  return new Promise((resolve, reject) => {
    const socket = net.createConnection({ host, port }, () => resolve(socket));
    socket.setNoDelay(true);
    socket.once('error', reject);
  });
}

function writeStatus(socket, status) {
  const payload = Object.assign({}, status, { ts_ms: Date.now() });
  socket.write(`${JSON.stringify(payload)}\n`);
}

function sendUdpFrame(udp, opts, modeName, frameId, frame) {
  const mode = MODE_BY_NAME[modeName];
  const chunkTotal = Math.max(1, Math.ceil(frame.image.length / DEFAULT_CHUNK_BYTES));
  for (let idx = 0; idx < chunkTotal; idx += 1) {
    const start = idx * DEFAULT_CHUNK_BYTES;
    const payload = frame.image.subarray(start, start + DEFAULT_CHUNK_BYTES);
    const packet = Buffer.alloc(HEADER_SIZE + payload.length);
    packet.writeUInt32BE(MAGIC, 0);
    packet.writeUInt32BE(frameId >>> 0, 4);
    packet.writeUInt16BE(idx, 8);
    packet.writeUInt16BE(chunkTotal, 10);
    packet.writeUInt16BE(payload.length, 12);
    packet.writeUInt16BE(frame.width, 14);
    packet.writeUInt16BE(frame.height, 16);
    packet.writeUInt8(mode, 18);
    packet.writeUInt8(frame.format, 19);
    payload.copy(packet, HEADER_SIZE);
    udp.send(packet, opts.udpPort, opts.host);
  }
}

function frameForSample(sample, modeName, seq) {
  if (sample && sample.frames && sample.frames[modeName]) return sample.frames[modeName];
  return syntheticImage(modeName, seq);
}

async function main() {
  const opts = parseArgs(process.argv.slice(2));
  const fixture = loadFixture(opts.fixtureDir, opts.modes);
  const udp = dgram.createSocket('udp4');
  const tcp = await connectTcp(opts.host, opts.tcpPort);
  const startedAt = Date.now();
  const intervalMs = Math.max(1, Math.round(1000 / opts.fps));
  let seq = 0;
  let frameId = 1;

  console.log(`[mock_board] tcp=${opts.host}:${opts.tcpPort} udp=${opts.host}:${opts.udpPort}`);
  console.log(`[mock_board] source=${fixture ? opts.fixtureDir : 'synthetic'} modes=${opts.statusOnly ? 'status-only' : opts.modes.join(',')}`);

  const cleanup = () => {
    try { tcp.end(); } catch (_) {}
    try { udp.close(); } catch (_) {}
  };
  process.on('SIGINT', () => {
    cleanup();
    process.exit(0);
  });

  while (true) {
    const sample = fixture ? fixture[seq % fixture.length] : null;
    const status = sample && sample.status ? sample.status : makeSyntheticStatus(seq);
    writeStatus(tcp, status);
    if (!opts.statusOnly) {
      for (const modeName of opts.modes) {
        sendUdpFrame(udp, opts, modeName, frameId, frameForSample(sample, modeName, seq));
        frameId = (frameId + 1) >>> 0;
      }
    }

    seq += 1;
    process.stdout.write(`\r[mock_board] sent=${seq} frame_id=${frameId}`);
    if (opts.once) break;
    if (opts.durationSec > 0 && Date.now() - startedAt >= opts.durationSec * 1000) break;

    let delayMs = intervalMs;
    if (fixture && fixture.length > 1) {
      const current = fixture[(seq - 1) % fixture.length];
      const next = fixture[seq % fixture.length];
      const dt = Number(next.tMs) - Number(current.tMs);
      delayMs = Number.isFinite(dt) && dt > 0 ? dt : intervalMs;
    }
    await sleep(delayMs);
  }

  process.stdout.write('\n');
  await sleep(100);
  cleanup();
}

main().catch((err) => {
  console.error(`[mock_board] ${err && err.stack ? err.stack : err}`);
  process.exit(1);
});
