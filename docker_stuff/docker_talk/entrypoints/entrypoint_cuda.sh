
cd ${ROBOT_CODE}/whispercpp/ && cmake -B build -S . -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=${HOME}/userInstalled \
    -DGGML_CUDA=ON && \
    cmake --build build -j8 && cmake --build build --target install
cd ${ROBOT_CODE}/yarp-device-speechTranscription-whisper && \
    cmake -B build -S . -DCMAKE_BUILD_TYPE=Release -Dwhisper_DIR=${HOME}/userInstalled && \
    cmake --build build -j8
cd ${ROBOT_CODE}/yarp-device-llama2 && \
    cmake -B build -S . -DCMAKE_BUILD_TYPE=Release -DLLAMA_ALL_WARNINGS=ON \
    -DGGML_CUDA=ON -DLLAMA_BUILD_COMMON=ON \
    -DLLAMA_BUILD_EXAMPLES=ON -DLLAMA_BUILD_SERVER=ON && \
    cmake --build build -j8

exec "$@"
