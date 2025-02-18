#!/usr/bin/env python3
import yarp
import numpy as np
from openwakeword.model import Model
import openwakeword
import multiprocessing as mp
import threading
import queue
from dataclasses import dataclass
from typing import List, Dict
import argparse
import os


@dataclass
class WakeWordConfig:
    model_paths: List[str]
    inference_framework: str = "onnx"
    buffer_size: int = 1280
    sample_rate: int = 16000
    detection_thresholds: Dict[str, float] = None

    def __post_init__(self):
        if self.detection_thresholds is None:
            # Default threshold of 0.15 for all models
            self.detection_thresholds = {
                os.path.splitext(os.path.basename(model_path))[0]: 0.15
                for model_path in self.model_paths
            }

def parse_args():
    parser = argparse.ArgumentParser(description='Wake Word Detection Module')
    
    # Model configuration
    parser.add_argument('--models', nargs='+', default=['./hey_r_one.onnx'],
                      help='Paths to wake word model files')
    parser.add_argument('--framework', default='onnx',
                      help='Inference framework to use (default: onnx)')
    
    # Audio configuration
    parser.add_argument('--buffer-size', type=int, default=1280,
                      help='Audio buffer size (default: 1280)')
    parser.add_argument('--sample-rate', type=int, default=16000,
                      help='Audio sample rate in Hz (default: 16000)')
    
    # Detection configuration
    parser.add_argument('--thresholds', type=float, nargs='+',
                      help='List of thresholds matching the order of models (default: 0.2 for each)')
    
    args = parser.parse_args()
    
    # Create threshold dictionary by matching models with thresholds
    if args.thresholds:
        if len(args.thresholds) != len(args.models):
            print(f"Warning: Number of thresholds ({len(args.thresholds)}) "
                  f"doesn't match number of models ({len(args.models)}). "
                  "Using default threshold 0.2 for remaining models.")
        
        thresholds = {}
        for model_path, threshold in zip(args.models, args.thresholds + [0.2] * len(args.models)):
            model_name = os.path.splitext(os.path.basename(model_path))[0]
            thresholds[model_name] = threshold
    else:
        thresholds = None
    
    return WakeWordConfig(
        model_paths=args.models,
        inference_framework=args.framework,
        buffer_size=args.buffer_size,
        sample_rate=args.sample_rate,
        detection_thresholds=thresholds
    )

def model_inference_process(sound_queue, result_queue, control_queue, config):
    model = Model(wakeword_models=config.model_paths, inference_framework=config.inference_framework)
    while True:
        buffer = sound_queue.get()
        if buffer is None:
            break
            
        try:
            while not control_queue.empty():
                signal = control_queue.get_nowait()
                if signal == "RESET":
                    model.reset()
        except queue.Empty:
            pass
            
        try:
            pred = model.predict(buffer)
            result_queue.put((pred, buffer))
        except Exception as e:
            result_queue.put(f"Error: {e}")

class OWWCallback(yarp.TypedReaderCallbackSound):
    def __init__(self, sound_queue, out_port, config):
        super().__init__()
        self.queue = sound_queue
        self.out_port = out_port
        self.config = config
        self.buffer = np.zeros(config.buffer_size, dtype=np.int16)
        self.fill_counter = 0
        self.active = True

    def reset_state(self):
        self.fill_counter = 0

    def onRead(self, sound, reader):
        if not self.active:
            if self.out_port is not None:
                out_sound = self.out_port.prepare()
                out_sound.resize(sound.getSamples())
                out_sound.setFrequency(self.config.sample_rate)
                for j in range(out_sound.getSamples()):
                    out_sound.set(sound.get(j), j)
                self.out_port.write()
            return
        
        try:
            num_samples = sound.getSamples()
            for i in range(num_samples):
                self.buffer[self.fill_counter] = sound.get(i)
                self.fill_counter += 1
                if self.fill_counter == self.config.buffer_size:
                    self.queue.put(self.buffer.copy())
                    self.fill_counter = 0
        except Exception as e:
            print(f"Error in onRead: {e}")

def clear_queue(q):
    while not q.empty():
        try:
            q.get_nowait()
        except queue.Empty:
            break

def rpc_command_listener(rpc_port, face_port, callback, control_queue):
    while True:
        cmd_bottle = yarp.Bottle()
        if not rpc_port.read(cmd_bottle, True):
            continue
        try:
            cmd_str = cmd_bottle.get(0).asString().strip().lower()
        except Exception as e:
            rpc_port.reply(yarp.Bottle("invalid command"))
            continue
        
        if cmd_str == "stop":
            callback.active = True
            callback.reset_state()
            clear_queue(callback.queue)
            control_queue.put("RESET")
            color_eyes(face_port, 255, 255, 255)
            rpc_port.reply(yarp.Bottle("streaming paused"))
            print("Streaming paused and inference started via RPC command.")
        else:
            rpc_port.reply(yarp.Bottle("unknown command"))
            print(f"Received unknown RPC command: {cmd_str}")

def color_eyes(face_port, r, g, b):
    face_bottle = face_port.prepare()
    face_bottle.clear()
    face_bottle.addString("notify_eyes")
    face_bottle.addFloat32(r)
    face_bottle.addFloat32(g)
    face_bottle.addFloat32(b)
    face_port.write()
            
def notify_detection(notification_port, face_port):
    color_eyes(face_port, 0, 255, 255)
    
    notification_bottle = notification_port.prepare()
    notification_bottle.clear()
    notification_bottle.addString("play_sound")
    notification_port.write()

def main():
    openwakeword.utils.download_models()
    config = parse_args()
    print("Running with configuration:", config)

    yarp.Network.init()

    sound_queue = mp.Queue()
    result_queue = mp.Queue()
    control_queue = mp.Queue()

    process = mp.Process(
        target=model_inference_process, 
        args=(sound_queue, result_queue, control_queue, config)
    )
    process.start()

    out_sound_port = yarp.BufferedPortSound()
    out_sound_port.open("/wake/audio:o")
    in_sound_port = yarp.BufferedPortSound()
    in_sound_port.open("/wake/audio:i")
    
    notification_port = yarp.BufferedPortBottle()
    notification_port.open("/wake/notification:o")
    face_port = yarp.BufferedPortBottle()
    face_port.open("/wake/face:o")
    
    callback = OWWCallback(sound_queue, out_sound_port, config)
    in_sound_port.useCallback(callback)

    rpc_port = yarp.RpcServer()
    rpc_port.open("/wake/rpc:i")
    rpc_thread = threading.Thread(
        target=rpc_command_listener, 
        args=(rpc_port, face_port, callback, control_queue)
    )
    rpc_thread.daemon = True
    rpc_thread.start()

    try:
        while True:
            try:
                result = result_queue.get(timeout=0.01)
                print(f"Prediction: {result}")
                if not callback.active:
                    continue
                if isinstance(result, tuple):
                    for model_name, threshold in config.detection_thresholds.items():
                        if result[0][model_name] > threshold:
                            callback.active = False
                            callback.reset_state()
                            clear_queue(sound_queue)
                            control_queue.put("RESET")
                            notify_detection(notification_port, face_port)
                            break
            except queue.Empty:
                continue
    except KeyboardInterrupt:
        print("KeyboardInterrupt received; exiting.")
    finally:
        sound_queue.put(None)
        process.join()
        in_sound_port.close()
        out_sound_port.close()
        notification_port.close()
        face_port.close()
        rpc_port.close()
        yarp.Network.fini()

if __name__ == '__main__':
    main()
