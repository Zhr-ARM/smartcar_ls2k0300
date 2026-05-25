#!/usr/bin/env node
'use strict';

const fs = require('fs');
const path = require('path');
const crypto = require('crypto');

const ROOT_DIR = path.resolve(__dirname, '..');
const DEFAULT_OUT_ROOT = path.join(ROOT_DIR, 'recordings');
const DEFAULT_BASE_URL = 'http://127.0.0.1:9090';
const FRAME_ENDPOINTS = {
  gray: '/api/frame_gray.jpg',
  rgb: '/api/frame_rgb.jpg',
  binary: '/api/frame_binary.jpg',
  roi64: '/api/frame_roi64.jpg'
};

function usage() {
  console.log(`Usage:
  node scripts/record_live_fixture.js [options]

Options:
  --base <url>        Receiver base URL, default ${DEFAULT_BASE_URL}
  --duration <sec>    Capture duration in seconds, default 10
  --interval <ms>     Capture interval in milliseconds, default 200
  --out <dir>         Output directory, default recordings/live_fixture_<timestamp>
  --modes <list>      Comma-separated frame modes, default gray,rgb,binary,roi64
  --status-only       Capture /api/status only
  --help              Show this help
`);
}

function parseArgs(argv) {
  const opts = {
    base: DEFAULT_BASE_URL,
    durationSec: 10,
    intervalMs: 200,
    outDir: '',
    modes: Object.keys(FRAME_ENDPOINTS),
    statusOnly: false
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
    const next = argv[i + 1];
    if (arg === '--base') {
      opts.base = String(next || '').replace(/\/+$/, '');
      i += 1;
    } else if (arg === '--duration') {
      opts.durationSec = Number(next);
      i += 1;
    } else if (arg === '--interval') {
      opts.intervalMs = Number(next);
      i += 1;
    } else if (arg === '--out') {
      opts.outDir = String(next || '');
      i += 1;
    } else if (arg === '--modes') {
      opts.modes = String(next || '')
        .split(',')
        .map((item) => item.trim())
        .filter(Boolean);
      i += 1;
    } else {
      throw new Error(`Unknown option: ${arg}`);
    }
  }

  if (!opts.base) throw new Error('--base cannot be empty');
  if (!Number.isFinite(opts.durationSec) || opts.durationSec <= 0) throw new Error('--duration must be > 0');
  if (!Number.isFinite(opts.intervalMs) || opts.intervalMs < 50) throw new Error('--interval must be >= 50');
  const invalidModes = opts.modes.filter((mode) => !FRAME_ENDPOINTS[mode]);
  if (invalidModes.length > 0) throw new Error(`Unknown frame mode(s): ${invalidModes.join(', ')}`);
  return opts;
}

function timestampForFolder(date) {
  return date.toISOString().replace(/[-:]/g, '').replace(/\.\d{3}Z$/, 'Z');
}

function sleep(ms) {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

function errorMessage(err) {
  if (!err) return 'unknown error';
  const base = err.message ? String(err.message) : String(err);
  if (err.cause && err.cause.message) return `${base}: ${err.cause.message}`;
  if (err.cause) return `${base}: ${String(err.cause)}`;
  return base;
}

function sha256(buffer) {
  return crypto.createHash('sha256').update(buffer).digest('hex');
}

function extForContentType(contentType) {
  const mime = String(contentType || '').split(';')[0].trim().toLowerCase();
  if (mime === 'image/png') return '.png';
  if (mime === 'image/webp') return '.webp';
  if (mime === 'image/bmp') return '.bmp';
  return '.jpg';
}

async function fetchJson(url) {
  const response = await fetch(url, { cache: 'no-store' });
  const text = await response.text();
  if (!response.ok) {
    throw new Error(`${response.status} ${response.statusText}: ${text.slice(0, 160)}`);
  }
  return JSON.parse(text || '{}');
}

async function fetchFrame(baseUrl, mode, sampleIndex, framesDir, seen) {
  const response = await fetch(`${baseUrl}${FRAME_ENDPOINTS[mode]}`, { cache: 'no-store' });
  const contentType = response.headers.get('content-type') || '';
  const buffer = Buffer.from(await response.arrayBuffer());
  const info = {
    ok: response.ok && contentType.startsWith('image/'),
    status_code: response.status,
    content_type: contentType,
    bytes: buffer.length
  };

  if (!info.ok) {
    info.error = buffer.toString('utf8').slice(0, 160);
    return info;
  }

  const digest = sha256(buffer);
  const ext = extForContentType(contentType);
  let file = seen.get(digest);
  if (!file) {
    file = path.join('frames', `${String(sampleIndex).padStart(5, '0')}_${mode}_${digest.slice(0, 10)}${ext}`);
    fs.writeFileSync(path.join(path.dirname(framesDir), file), buffer);
    seen.set(digest, file);
  }
  info.sha256 = digest;
  info.file = file;
  return info;
}

async function captureSample(opts, sampleIndex, startedAtMs, framesDir, seenFrames) {
  const nowMs = Date.now();
  const sample = {
    index: sampleIndex,
    t_ms: nowMs - startedAtMs,
    client_ts_ms: nowMs,
    status: null,
    frames: {}
  };

  try {
    sample.status = await fetchJson(`${opts.base}/api/status`);
  } catch (err) {
    sample.status_error = errorMessage(err);
  }

  if (!opts.statusOnly) {
    for (const mode of opts.modes) {
      try {
        sample.frames[mode] = await fetchFrame(opts.base, mode, sampleIndex, framesDir, seenFrames);
      } catch (err) {
        sample.frames[mode] = { ok: false, error: errorMessage(err) };
      }
    }
  }

  return sample;
}

async function main() {
  if (typeof fetch !== 'function') {
    throw new Error('This script requires Node.js 18+ because it uses built-in fetch().');
  }

  const opts = parseArgs(process.argv.slice(2));
  const startedAt = new Date();
  const outDir = opts.outDir
    ? path.resolve(opts.outDir)
    : path.join(DEFAULT_OUT_ROOT, `live_fixture_${timestampForFolder(startedAt)}`);
  const framesDir = path.join(outDir, 'frames');
  fs.mkdirSync(framesDir, { recursive: true });

  const startedAtMs = startedAt.getTime();
  const endAtMs = startedAtMs + Math.round(opts.durationSec * 1000);
  const samples = [];
  const seenFrames = new Map();
  let sampleIndex = 0;

  console.log(`[record_live_fixture] base=${opts.base}`);
  console.log(`[record_live_fixture] out=${outDir}`);
  console.log(`[record_live_fixture] duration=${opts.durationSec}s interval=${opts.intervalMs}ms modes=${opts.statusOnly ? 'status-only' : opts.modes.join(',')}`);

  while (Date.now() < endAtMs) {
    const tickStart = Date.now();
    const sample = await captureSample(opts, sampleIndex, startedAtMs, framesDir, seenFrames);
    samples.push(sample);
    const statusOk = sample.status ? 'status' : 'status!';
    const frameOkCount = Object.values(sample.frames).filter((item) => item && item.ok).length;
    process.stdout.write(`\r[record_live_fixture] samples=${samples.length} ${statusOk} frames=${frameOkCount}/${Object.keys(sample.frames).length}`);
    sampleIndex += 1;
    const elapsed = Date.now() - tickStart;
    await sleep(Math.max(0, opts.intervalMs - elapsed));
  }
  process.stdout.write('\n');

  const finishedAtMs = Date.now();
  const statusPayload = {
    recorded_at_ms: startedAtMs,
    duration_ms: finishedAtMs - startedAtMs,
    frame_count: samples.length,
    source: {
      kind: 'live_fixture',
      base_url: opts.base,
      interval_ms: opts.intervalMs,
      modes: opts.statusOnly ? [] : opts.modes
    },
    statuses: samples.map((sample) => ({
      client_ts_ms: sample.client_ts_ms,
      t_ms: sample.t_ms,
      status: sample.status,
      status_error: sample.status_error || undefined
    })),
    samples: samples.map((sample) => ({
      index: sample.index,
      t_ms: sample.t_ms,
      client_ts_ms: sample.client_ts_ms,
      status_error: sample.status_error || undefined,
      frames: sample.frames
    }))
  };
  const meta = {
    folder: path.basename(outDir),
    saved_at_ms: finishedAtMs,
    recorded_at_ms: startedAtMs,
    duration_ms: finishedAtMs - startedAtMs,
    frame_count: samples.length,
    unique_frame_count: seenFrames.size,
    source: statusPayload.source
  };

  fs.writeFileSync(path.join(outDir, 'status.json'), JSON.stringify(statusPayload, null, 2), 'utf8');
  fs.writeFileSync(path.join(outDir, 'meta.json'), JSON.stringify(meta, null, 2), 'utf8');
  console.log(`[record_live_fixture] wrote ${samples.length} samples, ${seenFrames.size} unique frames`);
}

main().catch((err) => {
  console.error(`[record_live_fixture] ${err && err.stack ? err.stack : err}`);
  process.exit(1);
});
