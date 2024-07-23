## This is a work in progress, such definition may change in the future once the functionality is fully implemented.

## Position based roaming

The modules in this folder allow the robot to roam between Access Points (AP) of which the position is physically known.

### Requirements

#### Wpa supplicant
- *On your host machine* You need a copy of `wpa_supplicant` (you can find the latest release [here](https://w1.fi/wpa_supplicant/)). Unzip the archive and put the source code in the `/opt` folder.
- `cd /opt/wpa_supplicant-2.* && make -C wpa_supplicant libwpa_client.so`

#### YARP
- Make sure YARP>3.9 is installed on your host machine

### Setup

- Before running your nav2 stack, make sure to include the positions of the APs in the locations.ini file. You can give the location any name that you want. Then you need to map the name used in the locations.ini file to the name of the mac address of the AP. You need to do this in a txt file of the style of `locations_ap_map.txt`. You can then run the `roaming_server` module with the `--loc_to_ap_map` option.

### Usage

- Run the `roaming_server` module on your preferred machine (the advice is to run it within the tour-guide-robot docker on the robot). This module is responsible for deciding whether to roam based on the position of the robot compared to the known access points. For now it uses the Euclidean distance to decide whether to roam or not.

- Run the `roaming_client` module on the host machine of the robot. This module is responsible for receiving the AP to roam to from the `roaming_server` and then connecting to the AP.
