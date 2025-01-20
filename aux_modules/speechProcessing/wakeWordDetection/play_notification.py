#!/usr/bin/env python3
import yarp
import sys
import subprocess

class SoundPlayerModule(yarp.RFModule):
    def configure(self, rf):
        self.port = yarp.BufferedPortBottle()
        self.port.open("/wake_notification:i")
        self.filename = rf.check("file", yarp.Value("notification.wav")).asString()
        print("Listening on port '/notification:i'...")
        return True

    def updateModule(self):
        bottle = self.port.read(True)  # Use non-blocking read
        if bottle is not None:
            command = bottle.get(0).toString()

            if command == "play_sound":
                print(f"Playing sound: {self.filename}")
                subprocess.run(["aplay", self.filename])
        return True

    def close(self):
        self.port.close()
        yarp.Network.fini()
        return True

if __name__ == "__main__":
    yarp.Network.init()
    module = SoundPlayerModule()

    rf = yarp.ResourceFinder()
    rf.setVerbose(True)
    rf.configure(sys.argv)

    module.runModule(rf)