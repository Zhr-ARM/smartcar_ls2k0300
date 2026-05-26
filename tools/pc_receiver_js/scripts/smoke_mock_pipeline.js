#!/usr/bin/env node
'use strict';

const { spawn } = require('node:child_process');
const fs = require('node:fs');
const path = require('node:path');

const ROOT_DIR = path.resolve(__dirname, '..');
const DEFAULT_HTTP_PORT = 19190;
const DEFAULT_UDP_PORT = 19100;
const DEFAULT_TCP_PORT = 19101;

function usage() {
  console.log(`Usage:
  node scripts/smoke_mock_pipeline.js [options]

Options:
  --host <ip>         Bind/connect host, default 127.0.0.1
  --http-port <port>  Test HTTP port, default ${DEFAULT_HTTP_PORT}
  --udp-port <port>   Test UDP port, default ${DEFAULT_UDP_PORT}
  --tcp-port <port>   Test TCP port, default ${DEFAULT_TCP_PORT}
  --fixture <dir>     Replay this fixture instead of auto-selecting the latest one
  --synthetic         Force generated mock data, even when recordings exist
  --timeout <ms>      Overall timeout, default 10000
  --help              Show this help
`);
}

function parseArgs(argv) {
  const opts = {
    host: '127.0.0.1',
    httpPort: DEFAULT_HTTP_PORT,
    udpPort: DEFAULT_UDP_PORT,
    tcpPort: DEFAULT_TCP_PORT,
    fixtureDir: '',
    synthetic: false,
    timeoutMs: 10000
  };

  for (let i = 0; i < argv.length; i += 1) {
    const arg = argv[i];
    if (arg === '--help' || arg === '-h') {
      usage();
      process.exit(0);
    }
    if (arg === '--synthetic' || arg === '--no-fixture') {
      opts.synthetic = true;
      continue;
    }
    const next = argv[i + 1];
    if (arg === '--host') {
      opts.host = String(next || '').trim();
      i += 1;
    } else if (arg === '--http-port') {
      opts.httpPort = Number(next);
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
    } else if (arg === '--timeout') {
      opts.timeoutMs = Number(next);
      i += 1;
    } else {
      throw new Error(`Unknown option: ${arg}`);
    }
  }

  if (!opts.host) throw new Error('--host cannot be empty');
  if (opts.synthetic && opts.fixtureDir) throw new Error('--synthetic cannot be combined with --fixture');
  for (const key of ['httpPort', 'udpPort', 'tcpPort']) {
    if (!Number.isInteger(opts[key]) || opts[key] <= 0) throw new Error(`--${key} must be a positive integer`);
  }
  if (!Number.isFinite(opts.timeoutMs) || opts.timeoutMs < 1000) throw new Error('--timeout must be >= 1000');
  return opts;
}

function sleep(ms) {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

function spawnNode(args, env = {}) {
  const child = spawn(process.execPath, args, {
    cwd: ROOT_DIR,
    env: Object.assign({}, process.env, env),
    stdio: ['ignore', 'pipe', 'pipe']
  });
  child.stdout.on('data', (chunk) => process.stdout.write(chunk));
  child.stderr.on('data', (chunk) => process.stderr.write(chunk));
  return child;
}

function stopChild(child) {
  if (!child || child.killed) return;
  child.kill('SIGTERM');
}

function findLatestFixture(recordingsDir) {
  if (!fs.existsSync(recordingsDir)) return '';
  const candidates = fs.readdirSync(recordingsDir, { withFileTypes: true })
    .filter((entry) => entry.isDirectory() && entry.name.startsWith('live_fixture_'))
    .map((entry) => {
      const fixtureDir = path.join(recordingsDir, entry.name);
      const statusPath = path.join(fixtureDir, 'status.json');
      if (!fs.existsSync(statusPath)) return null;
      const stat = fs.statSync(fixtureDir);
      return { fixtureDir, name: entry.name, mtimeMs: stat.mtimeMs };
    })
    .filter(Boolean)
    .sort((a, b) => b.name.localeCompare(a.name) || b.mtimeMs - a.mtimeMs);
  return candidates.length > 0 ? candidates[0].fixtureDir : '';
}

function selectFixture(opts) {
  if (opts.synthetic) return '';
  if (opts.fixtureDir) return opts.fixtureDir;
  return findLatestFixture(path.join(ROOT_DIR, 'recordings'));
}

async function fetchText(url) {
  const response = await fetch(url, { cache: 'no-store' });
  const text = await response.text();
  if (!response.ok) throw new Error(`${url} -> ${response.status}: ${text.slice(0, 120)}`);
  return { response, text };
}

async function waitForHttp(baseUrl, deadlineMs) {
  let lastError = null;
  while (Date.now() < deadlineMs) {
    try {
      await fetchText(`${baseUrl}/api/status`);
      return;
    } catch (err) {
      lastError = err;
      await sleep(150);
    }
  }
  throw new Error(`receiver HTTP not ready: ${lastError && lastError.message ? lastError.message : lastError}`);
}

async function waitForMockData(baseUrl, deadlineMs, source) {
  let lastStatus = null;
  while (Date.now() < deadlineMs) {
    const statusResult = await fetchText(`${baseUrl}/api/status`);
    lastStatus = JSON.parse(statusResult.text || '{}');
    const frameResult = await fetch(`${baseUrl}/api/frame_gray.jpg`, { cache: 'no-store' });
    const contentType = frameResult.headers.get('content-type') || '';
    const frameBytes = Buffer.from(await frameResult.arrayBuffer()).length;
    const statusReady = source === 'synthetic'
      ? lastStatus.mock_board === 1
      : lastStatus && Object.keys(lastStatus).length > 0;
    if (statusReady && frameResult.ok && contentType.startsWith('image/') && frameBytes > 100) {
      return { status: lastStatus, frame: { contentType, frameBytes } };
    }
    await sleep(150);
  }
  throw new Error(`mock data not visible through API from ${source}; last mock_board=${lastStatus && lastStatus.mock_board}`);
}

async function main() {
  if (typeof fetch !== 'function') {
    throw new Error('This script requires Node.js 18+ because it uses built-in fetch().');
  }
  const opts = parseArgs(process.argv.slice(2));
  const fixtureDir = selectFixture(opts);
  const source = fixtureDir ? 'fixture' : 'synthetic';
  const baseUrl = `http://${opts.host}:${opts.httpPort}`;
  const deadlineMs = Date.now() + opts.timeoutMs;
  let server = null;
  let mock = null;

  try {
    server = spawnNode(['server.js'], {
      BIND_HOST: opts.host,
      HTTP_PORT: String(opts.httpPort),
      UDP_PORT: String(opts.udpPort),
      TCP_PORT: String(opts.tcpPort)
    });
    await waitForHttp(baseUrl, deadlineMs);

    const mockArgs = [
      path.join('scripts', 'mock_board_sender.js'),
      '--host', opts.host,
      '--udp-port', String(opts.udpPort),
      '--tcp-port', String(opts.tcpPort),
      '--duration', '3',
      '--fps', '8'
    ];
    if (fixtureDir) mockArgs.push('--fixture', fixtureDir);
    console.log(`[smoke_mock_pipeline] source=${fixtureDir ? path.relative(ROOT_DIR, fixtureDir) : 'synthetic'}`);
    mock = spawnNode(mockArgs);

    const result = await waitForMockData(baseUrl, deadlineMs, source);
    const page = await fetchText(`${baseUrl}/`);
    if (!page.text.includes('/shared_receiver_core.js')) {
      throw new Error('main page loaded, but expected frontend script marker is missing');
    }
    const statusMarker = source === 'synthetic'
      ? `mock_seq=${result.status.mock_seq}`
      : `status_keys=${Object.keys(result.status).length}`;
    console.log(`\n[smoke_mock_pipeline] ok source=${source} ${statusMarker} frame=${result.frame.contentType} ${result.frame.frameBytes} bytes`);
  } finally {
    stopChild(mock);
    stopChild(server);
  }
}

main().catch((err) => {
  console.error(`[smoke_mock_pipeline] ${err && err.stack ? err.stack : err}`);
  process.exit(1);
});
