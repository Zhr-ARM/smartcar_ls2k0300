# WASM Output Directory

把 Emscripten 产物放到这个目录：

- `vision_pipeline.js`
- `vision_pipeline.wasm`

推荐构建方式放在：

- `tools/pc_receiver_local_compute/build_wasm.sh`

worker 会从固定路径加载：

- `/wasm/vision_pipeline.js`
