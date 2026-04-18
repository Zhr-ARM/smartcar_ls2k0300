#!/usr/bin/env node

const fs = require('fs');
const path = require('path');

function fail(message) {
  console.error(message);
  process.exit(1);
}

const presetFile = path.resolve(process.argv[2] || './connection_presets.json');
const presetId = process.argv[3];
const tomlFile = path.resolve(process.argv[4] || './smartcar_config.toml');

if (!presetId) {
  fail('缺少 preset id');
}
if (!fs.existsSync(presetFile)) {
  fail(`找不到预设文件: ${presetFile}`);
}
if (!fs.existsSync(tomlFile)) {
  fail(`找不到 TOML 文件: ${tomlFile}`);
}

const store = JSON.parse(fs.readFileSync(presetFile, 'utf8'));
if (!store || !Array.isArray(store.presets)) {
  fail('connection_presets.json 缺少 presets');
}

const preset = store.presets.find((item) => item.id === presetId);
if (!preset) {
  fail(`未找到预设: ${presetId}`);
}

const pcReceiverIp = String(preset.pc_receiver_ip || '').trim();
const assistantReceiverIp = String(preset.assistant_receiver_ip || pcReceiverIp).trim();
if (!pcReceiverIp) {
  fail(`预设 ${presetId} 缺少 pc_receiver_ip`);
}
if (!assistantReceiverIp) {
  fail(`预设 ${presetId} 缺少 assistant_receiver_ip`);
}

const tomlText = fs.readFileSync(tomlFile, 'utf8');
if (!tomlText.trim()) {
  fail(`TOML 文件为空: ${tomlFile}（可能是之前磁盘满导致写入中断）`);
}
const lines = tomlText.split('\n');
let section = '';
let replacedWeb = false;
let replacedAssistant = false;

for (let i = 0; i < lines.length; i += 1) {
  const rawLine = lines[i];
  const trimmed = rawLine.trim();
  if (trimmed.startsWith('[') && trimmed.endsWith(']')) {
    section = trimmed.slice(1, -1).trim();
    continue;
  }
  if (!trimmed.startsWith('server_ip')) {
    continue;
  }
  if (section === 'vision.runtime.web') {
    lines[i] = rawLine.replace(/server_ip\s*=\s*"[^"]*"/, `server_ip = "${pcReceiverIp}"`);
    replacedWeb = true;
    continue;
  }
  if (section === 'vision.runtime.assistant') {
    lines[i] = rawLine.replace(/server_ip\s*=\s*"[^"]*"/, `server_ip = "${assistantReceiverIp}"`);
    replacedAssistant = true;
  }
}

if (!replacedWeb) {
  fail('未找到 [vision.runtime.web] 下的 server_ip');
}
if (!replacedAssistant) {
  fail('未找到 [vision.runtime.assistant] 下的 server_ip');
}

const nextText = lines.join('\n');
const dir = path.dirname(tomlFile);
const base = path.basename(tomlFile);
const tmpFile = path.join(dir, `.${base}.tmp.${process.pid}.${Date.now()}`);
try {
  fs.writeFileSync(tmpFile, nextText, 'utf8');
  fs.renameSync(tmpFile, tomlFile);
} catch (err) {
  try {
    if (fs.existsSync(tmpFile)) {
      fs.unlinkSync(tmpFile);
    }
  } catch (_) {
    // ignore cleanup failure
  }
  fail(`写入 TOML 失败: ${err && err.message ? err.message : String(err)}`);
}
console.log(`已同步 preset=${presetId} 到 ${path.basename(tomlFile)}`);
