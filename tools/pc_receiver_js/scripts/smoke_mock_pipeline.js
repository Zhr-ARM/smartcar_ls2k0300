#!/usr/bin/env node
'use strict';

const { spawn } = require('node:child_process');
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
  --fixture <dir>     Replay fixture through mock board instead of synthetic data
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
    timeoutMs: 10000
  };

  for (let i = 0; i < argv.length; i += 1) {
    const arg = argv[i];
    if (arg === '--help' || arg === '-h') {
      usage();
      process.exit(0);
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

async function waitForMockData(baseUrl, deadlineMs) {
  let lastStatus = null;
  while (Date.now() < deadlineMs) {
    const statusResult = await fetchText(`${baseUrl}/api/status`);
    lastStatus = JSON.parse(statusResult.text || '{}');
    const frameResult = await fetch(`${baseUrl}/api/frame_gray.jpg`, { cache: 'no-store' });
    const contentType = frameResult.headers.get('content-type') || '';
    const frameBytes = Buffer.from(await frameResult.arrayBuffer()).length;
    if (lastStatus.mock_board === 1 && frameResult.ok && contentType.startsWith('image/') && frameBytes > 100) {
      return { status: lastStatus, frame: { contentType, frameBytes } };
    }
    await sleep(150);
  }
  throw new Error(`mock data not visible through API; last mock_board=${lastStatus && lastStatus.mock_board}`);
}

async function main() {
  if (typeof fetch !== 'function') {
    throw new Error('This script requires Node.js 18+ because it uses built-in fetch().');
  }
  const opts = parseArgs(process.argv.slice(2));
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
    if (opts.fixtureDir) mockArgs.push('--fixture', opts.fixtureDir);
    mock = spawnNode(mockArgs);

    const result = await waitForMockData(baseUrl, deadlineMs);
    const page = await fetchText(`${baseUrl}/`);
    if (!page.text.includes('/shared_receiver_core.js')) {
      throw new Error('main page loaded, but expected frontend script marker is missing');
    }
    console.log(`\n[smoke_mock_pipeline] ok status.mock_seq=${result.status.mock_seq} frame=${result.frame.contentType} ${result.frame.frameBytes} bytes`);
  } finally {
    stopChild(mock);
    stopChild(server);
  }
}

main().catch((err) => {
  console.error(`[smoke_mock_pipeline] ${err && err.stack ? err.stack : err}`);
  process.exit(1);
});
