#include "AudioCallback.h"
// #include "pv_recorder.h"

#include <iostream>
#include <vector>
#include <dlfcn.h>


static void *open_dl(const char *dl_path) {

#if defined(_WIN32) || defined(_WIN64)

    return LoadLibrary(dl_path);

#else

    return dlopen(dl_path, RTLD_NOW);

#endif
}

static void *load_symbol(void *handle, const char *symbol) {

#if defined(_WIN32) || defined(_WIN64)

    return GetProcAddress((HMODULE) handle, symbol);

#else

    return dlsym(handle, symbol);

#endif
}

static void close_dl(void *handle) {

#if defined(_WIN32) || defined(_WIN64)

    FreeLibrary((HMODULE) handle);

#else

    dlclose(handle);

#endif
}

static void print_dl_error(const char *message) {

#if defined(_WIN32) || defined(_WIN64)

    fprintf(stderr, "%s with code '%lu'.\n", message, GetLastError());

#else

    fprintf(stderr, "%s with '%s'.\n", message, dlerror());

#endif
}

AudioCallback::AudioCallback(std::string &rpcPortName){
    auto access_key = "E3HSLWAlzc76SFsflAy+9NSJotzp4u1VQIKU63sdiyc9CzqQL8HRDg==";
    auto model_path = "/home/user1/share/porcupine/lib/common/porcupine_params.pv";
    auto keyword_path = "/home/user1/share/porcupine_playground/hey-r-one_en_linux_v3_0_0.ppn";
    float sensitivity = 0.5;

    m_rpcClientPort.open(rpcPortName);
    m_rpcClient.yarp().attachAsClient(m_rpcClientPort);

    m_audioOut.open(m_audioOutName);
    
    pv_status_t porcupine_status = pv_porcupine_init(access_key, model_path, 1, &keyword_path, &sensitivity, &m_porcupine);
    auto model_status = pv_status_to_string(porcupine_status);

    std::cout << "Model status: " << model_status << std::endl;
}

void AudioCallback::onRead(yarp::sig::Sound &soundReceived) {
    if (m_currentlyStreaming)
    {
        yarp::sig::Sound& soundToSend = m_audioOut.prepare();
        soundToSend = soundReceived;
        m_audioOut.write();
    }
    else {
        processFrame(soundReceived);
    }    
}

void AudioCallback::processFrame(yarp::sig::Sound &soundReceived) {
    size_t num_samples = soundReceived.getSamples();

    int slice = pv_porcupine_frame_length();
    int16_t audioFrame[slice];
    int slice_counter = 0;

    bool sendRemainingSlices = false; // once wake word is detected stop inferring, just accumulate
    std::vector<int16_t> remainingSamples;
    int remainingSamplesIdx = 0;

    for (size_t i = 0; i < num_samples; i++) {
        //accumulate all remaining samples for sending
        if (sendRemainingSlices) {
            remainingSamples.at(remainingSamplesIdx) = soundReceived.get(i);
            ++remainingSamplesIdx;
            continue;
        }
        
        audioFrame[slice_counter] = soundReceived.get(i);
        ++slice_counter;
        
        if (slice_counter == slice && !sendRemainingSlices) {
            int32_t keyword_index = -1;
            pv_status_t status = pv_porcupine_process(m_porcupine, audioFrame, &keyword_index);

            auto porcupine_status = pv_status_to_string(status);

            if (keyword_index != -1) {
                std::cout <<  "keyword detected!!!!!!!!!!!" << std::endl;
                m_currentlyStreaming = true;
                sendRemainingSlices = true;
                remainingSamples.resize(num_samples - i - 1);
            }
            // else {
            //     std::cout <<  "no keyword :(" << std::endl;
            // }

            slice_counter = 0;
        }
    }

    // send any remaining slices after the wake word detected
    if (sendRemainingSlices && remainingSamples.size() > 0) {
        yarp::sig::Sound& soundToSend = m_audioOut.prepare();
        soundToSend.setFrequency(16000);
        soundToSend.resize(remainingSamples.size());
        for (size_t i = 0; i < remainingSamples.size(); i++)
        {
            soundToSend.set(remainingSamples.at(i), i);
        }
        m_audioOut.write();
    }
}