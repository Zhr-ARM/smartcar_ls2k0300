const fs = require('node:fs');
const path = require('node:path');
const crypto = require('node:crypto');

const TRACKED_RELATIVE_PATHS = [
  'tools/pc_receiver_local_compute/wasm_bridge/CMakeLists.txt',
  'tools/pc_receiver_local_compute/wasm_bridge/web_vision_bridge.h',
  'tools/pc_receiver_local_compute/wasm_bridge/web_vision_bridge.cpp',
  'tools/pc_receiver_local_compute/wasm_bridge/web_vision_abi.cpp',
  'tools/pc_receiver_local_compute/wasm_bridge/compat/line_follow_thread.h',
  'tools/pc_receiver_local_compute/wasm_bridge/compat/line_follow_thread.cpp',
  'tools/pc_receiver_local_compute/wasm_bridge/compat/vision_frame_capture.cpp',
  'tools/pc_receiver_local_compute/wasm_bridge/compat/driver/vision/vision_frame_capture.h',
  'tools/pc_receiver_local_compute/wasm_bridge/compat/driver/vision/vision_image_processor.h',
  'project/code/driver/vision/vision_config.h',
  'project/code/driver/vision/vision_config.c',
  'project/code/driver/vision/vision_line_error_layer.h',
  'project/code/driver/vision/vision_line_error_layer.cpp',
  'project/code/driver/vision/vision_image_processor.h',
  'project/code/driver/vision/vision_image_processor.cpp',
];

function readFileInfo(rootDir, relativePath) {
  const absolutePath = path.join(rootDir, relativePath);
  if (!fs.existsSync(absolutePath)) {
    return {
      relative_path: relativePath,
      exists: false,
      size: null,
      mtime_ms: null,
      sha256: null,
    };
  }

  const buffer = fs.readFileSync(absolutePath);
  const stat = fs.statSync(absolutePath);
  return {
    relative_path: relativePath,
    exists: true,
    size: stat.size,
    mtime_ms: Math.round(stat.mtimeMs),
    sha256: crypto.createHash('sha256').update(buffer).digest('hex'),
  };
}

function computeTrackedSources(rootDir) {
  const files = TRACKED_RELATIVE_PATHS.map((relativePath) => readFileInfo(rootDir, relativePath));
  const digest = crypto.createHash('sha256');
  for (const file of files) {
    digest.update(file.relative_path);
    digest.update('\0');
    digest.update(file.exists ? '1' : '0');
    digest.update('\0');
    if (file.exists) {
      digest.update(String(file.size));
      digest.update('\0');
      digest.update(file.sha256 || '');
    }
    digest.update('\0');
  }

  return {
    tracked_files: files,
    tracked_file_count: files.length,
    missing_files: files.filter((file) => !file.exists).map((file) => file.relative_path),
    source_digest: digest.digest('hex'),
  };
}

function buildWasmSyncMetadata(rootDir) {
  const computed = computeTrackedSources(rootDir);
  const now = new Date();
  return {
    schema_version: 1,
    built_at_iso: now.toISOString(),
    built_at_ms: now.getTime(),
    source_digest: computed.source_digest,
    tracked_file_count: computed.tracked_file_count,
    missing_files: computed.missing_files,
    tracked_files: computed.tracked_files,
  };
}

function loadBuiltMetadata(metadataPath) {
  if (!metadataPath || !fs.existsSync(metadataPath)) return null;
  try {
    return JSON.parse(fs.readFileSync(metadataPath, 'utf8'));
  } catch (_) {
    return null;
  }
}

function summarizeSyncStatus(rootDir, metadataPath) {
  const current = computeTrackedSources(rootDir);
  const built = loadBuiltMetadata(metadataPath);
  if (!built) {
    return {
      available: false,
      sync_state: 'missing',
      sync_label: '未构建',
      sync_detail: '还没有找到本地复算 WASM 的同步信息，请先重新执行 build_wasm.sh。',
      current_source_digest: current.source_digest,
      built_source_digest: null,
      built_at_iso: null,
      tracked_file_count: current.tracked_file_count,
      missing_files: current.missing_files,
      is_synced: false,
    };
  }

  const isSynced = built.source_digest === current.source_digest && current.missing_files.length === 0;
  return {
    available: true,
    sync_state: isSynced ? 'synced' : 'stale',
    sync_label: isSynced ? '已同步' : '未同步',
    sync_detail: isSynced
      ? '网页端本地复算使用的 WASM 与当前算法源码一致。'
      : '当前 project/ 算法源码与已构建的网页端 WASM 不一致，需要重新执行 build_wasm.sh。',
    current_source_digest: current.source_digest,
    built_source_digest: built.source_digest || null,
    built_at_iso: built.built_at_iso || null,
    tracked_file_count: current.tracked_file_count,
    missing_files: current.missing_files,
    is_synced: isSynced,
  };
}

function writeMetadataFile(rootDir, outputPath) {
  const metadata = buildWasmSyncMetadata(rootDir);
  fs.mkdirSync(path.dirname(outputPath), { recursive: true });
  fs.writeFileSync(outputPath, `${JSON.stringify(metadata, null, 2)}\n`, 'utf8');
  return metadata;
}

module.exports = {
  TRACKED_RELATIVE_PATHS,
  computeTrackedSources,
  buildWasmSyncMetadata,
  summarizeSyncStatus,
  writeMetadataFile,
};

if (require.main === module) {
  const args = process.argv.slice(2);
  const rootIndex = args.indexOf('--root');
  const writeIndex = args.indexOf('--write');
  const rootDir = rootIndex >= 0 && args[rootIndex + 1] ? path.resolve(args[rootIndex + 1]) : path.resolve(__dirname, '..', '..');
  if (writeIndex >= 0 && args[writeIndex + 1]) {
    const outputPath = path.resolve(args[writeIndex + 1]);
    const metadata = writeMetadataFile(rootDir, outputPath);
    process.stdout.write(`${JSON.stringify(metadata, null, 2)}\n`);
  } else {
    const summary = summarizeSyncStatus(rootDir, path.join(rootDir, 'tools/pc_receiver_js/public/wasm/vision_pipeline.sync.json'));
    process.stdout.write(`${JSON.stringify(summary, null, 2)}\n`);
  }
}
