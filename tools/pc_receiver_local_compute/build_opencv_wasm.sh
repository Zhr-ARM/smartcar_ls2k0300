#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/../.." && pwd)"
TOOLS_LOCAL_DIR="$ROOT_DIR/tools/.local"
EMSDK_DIR="$TOOLS_LOCAL_DIR/emsdk"
OPENCV_ROOT="$TOOLS_LOCAL_DIR/opencv-wasm"
OPENCV_SRC_DIR="$OPENCV_ROOT/src"
OPENCV_BUILD_DIR="$OPENCV_ROOT/build"
OPENCV_INSTALL_DIR="$OPENCV_ROOT/install"
OPENCV_TAG="${OPENCV_TAG:-4.5.4}"
BUILD_JOBS="${BUILD_JOBS:-$(nproc)}"

if [ -f "$EMSDK_DIR/emsdk_env.sh" ]; then
  # Allow the script to work even when the user starts from a fresh shell.
  # Keep the output quiet so repeated usage stays readable.
  export EMSDK_QUIET=1
  # shellcheck disable=SC1091
  source "$EMSDK_DIR/emsdk_env.sh" >/dev/null
fi

if ! command -v emcmake >/dev/null 2>&1 || ! command -v emcc >/dev/null 2>&1; then
  echo "Emscripten toolchain not found. Install tools/.local/emsdk or source emsdk_env.sh first."
  exit 1
fi

mkdir -p "$TOOLS_LOCAL_DIR"

if [ ! -d "$OPENCV_SRC_DIR/.git" ]; then
  git clone --branch "$OPENCV_TAG" --depth 1 https://github.com/opencv/opencv.git "$OPENCV_SRC_DIR"
else
  git -C "$OPENCV_SRC_DIR" fetch --depth 1 origin "refs/tags/$OPENCV_TAG:refs/tags/$OPENCV_TAG"
  git -C "$OPENCV_SRC_DIR" checkout --force "$OPENCV_TAG"
fi

emcmake cmake -S "$OPENCV_SRC_DIR" -B "$OPENCV_BUILD_DIR" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$OPENCV_INSTALL_DIR" \
  -DBUILD_SHARED_LIBS=OFF \
  -DBUILD_LIST=core,imgproc \
  -DCPU_BASELINE= \
  -DCPU_DISPATCH= \
  -DCV_ENABLE_INTRINSICS=OFF \
  -DBUILD_opencv_apps=OFF \
  -DBUILD_TESTS=OFF \
  -DBUILD_PERF_TESTS=OFF \
  -DBUILD_EXAMPLES=OFF \
  -DBUILD_PACKAGE=OFF \
  -DBUILD_JAVA=OFF \
  -DBUILD_FAT_JAVA_LIB=OFF \
  -DBUILD_DOCS=OFF \
  -DBUILD_OPENEXR=OFF \
  -DWITH_1394=OFF \
  -DWITH_ADE=OFF \
  -DWITH_CUDA=OFF \
  -DWITH_EIGEN=OFF \
  -DWITH_FFMPEG=OFF \
  -DWITH_GSTREAMER=OFF \
  -DWITH_GTK=OFF \
  -DWITH_IPP=OFF \
  -DWITH_ITT=OFF \
  -DWITH_JASPER=OFF \
  -DWITH_JPEG=OFF \
  -DWITH_OPENCL=OFF \
  -DWITH_OPENCLAMDBLAS=OFF \
  -DWITH_OPENCLAMDFFT=OFF \
  -DWITH_OPENEXR=OFF \
  -DWITH_OPENGL=OFF \
  -DWITH_OPENJPEG=OFF \
  -DWITH_PNG=OFF \
  -DWITH_QUIRC=OFF \
  -DWITH_TBB=OFF \
  -DWITH_TIFF=OFF \
  -DWITH_V4L=OFF \
  -DWITH_WEBP=OFF \
  -DZLIB_LIBRARY= \
  -DZLIB_INCLUDE_DIR=

cmake --build "$OPENCV_BUILD_DIR" -j"$BUILD_JOBS"
cmake --install "$OPENCV_BUILD_DIR"

PROTOBUF_PLACEHOLDER="$OPENCV_INSTALL_DIR/lib/opencv4/3rdparty/liblibprotobuf.a"
if [ ! -f "$PROTOBUF_PLACEHOLDER" ]; then
  if command -v emar >/dev/null 2>&1; then
    emar rcs "$PROTOBUF_PLACEHOLDER"
  else
    echo "Warning: emar not found, could not create $PROTOBUF_PLACEHOLDER placeholder."
  fi
fi

echo "WASM OpenCV installed to $OPENCV_INSTALL_DIR"
echo "Use OpenCV_DIR=$OPENCV_INSTALL_DIR/lib/cmake/opencv4 for Emscripten builds."
