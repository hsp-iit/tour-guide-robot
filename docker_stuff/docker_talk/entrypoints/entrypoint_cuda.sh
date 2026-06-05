#!/bin/sh
set -eu

ROBOT_CODE="${ROBOT_CODE:-/usr/local/src/robot}"
INSTALL_PREFIX="${HOME}/userInstalled"
STAMP_FILE="${CUDA_BUILD_STAMP:-${INSTALL_PREFIX}/.cuda-build.done}"
BUILD_JOBS="${CUDA_BUILD_JOBS:-8}"
CUDA_ARCHITECTURES="${CUDA_ARCHITECTURES:-native}"
CMAKE_PREFIX_PATH_VALUE="${INSTALL_PREFIX}"

if [ -n "${CMAKE_PREFIX_PATH:-}" ]; then
    CMAKE_PREFIX_PATH_VALUE="${CMAKE_PREFIX_PATH_VALUE};${CMAKE_PREFIX_PATH}"
fi

if [ -n "${YARP_DIR:-}" ]; then
    CMAKE_PREFIX_PATH_VALUE="${CMAKE_PREFIX_PATH_VALUE};${YARP_DIR}"
fi

if [ -n "${YCM_DIR:-}" ]; then
    CMAKE_PREFIX_PATH_VALUE="${CMAKE_PREFIX_PATH_VALUE};${YCM_DIR}"
fi

export CMAKE_PREFIX_PATH="${CMAKE_PREFIX_PATH_VALUE}"

run_command() {
    if [ "$#" -gt 0 ]; then
        exec "$@"
    fi

    exec bash
}

if [ "${CUDA_REBUILD:-0}" != "1" ] && [ -f "${STAMP_FILE}" ]; then
    echo "CUDA-dependent repositories already built. Set CUDA_REBUILD=1 to rebuild."
    run_command "$@"
fi

echo "Building CUDA-dependent repositories..."

sudo mkdir -p \
    "${INSTALL_PREFIX}" \
    "${ROBOT_CODE}/whisper.cpp/build" \
    "${ROBOT_CODE}/yarp-device-speechTranscription-whisper/build" \
    "${ROBOT_CODE}/yarp-device-llama2/build"

sudo chown -R "$(id -u):$(id -g)" \
    "${INSTALL_PREFIX}" \
    "${ROBOT_CODE}/whisper.cpp/build" \
    "${ROBOT_CODE}/yarp-device-speechTranscription-whisper/build" \
    "${ROBOT_CODE}/yarp-device-llama2/build"

rm -f "${STAMP_FILE}"

cd "${ROBOT_CODE}/whisper.cpp"
cmake -B build -S . \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}" \
    -DCMAKE_CUDA_ARCHITECTURES="${CUDA_ARCHITECTURES}" \
    -DGGML_CUDA=ON
cmake --build build --parallel "${BUILD_JOBS}"
cmake --build build --target install --parallel "${BUILD_JOBS}"

cd "${ROBOT_CODE}/yarp-device-speechTranscription-whisper"
cmake -B build -S . \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_PREFIX_PATH="${CMAKE_PREFIX_PATH_VALUE}" \
    -Dwhisper_DIR="${INSTALL_PREFIX}/lib/cmake/whisper" \
    -Dggml_DIR="${INSTALL_PREFIX}/lib/cmake/ggml" \
    -DBUILD_TESTING=OFF
cmake --build build --parallel "${BUILD_JOBS}"

cd "${ROBOT_CODE}/yarp-device-llama2"
cmake -B build -S . \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_PREFIX_PATH="${CMAKE_PREFIX_PATH_VALUE}" \
    -DCMAKE_CUDA_ARCHITECTURES="${CUDA_ARCHITECTURES}" \
    -DLLAMA_ALL_WARNINGS=ON \
    -DGGML_CUDA=ON \
    -DLLAMA_BUILD_COMMON=ON \
    -DLLAMA_BUILD_EXAMPLES=ON \
    -DLLAMA_BUILD_SERVER=ON \
    -DBUILD_TESTING=OFF
cmake --build build --parallel "${BUILD_JOBS}"

date -u +"%Y-%m-%dT%H:%M:%SZ" > "${STAMP_FILE}"

run_command "$@"
