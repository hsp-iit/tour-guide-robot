#!/bin/bash

# This is an utility to make the R1 robot roam more aggressively than it would do by default

if [ "$EUID" -ne 0 ]
  then echo "Please run the script like this:

sudo -E su # to keep the environment variables
roam.sh
"
  exit
fi


# Name of the SSID
ESSID=$(iwgetid -r)

# These frequencies are expressed in MHz and they are the ones used by the APs
FREQ=$AP_FREQUENCIES

if [ -z "$FREQ" ]
then
    echo 'Please set the AP_FREQUENCIES environment variable like "5000,5010,...,5040"'
    exit -1
fi

echo "Scanning only on these freqs: $FREQ Mhz"

SLEEP_DURATION=4
DETECTION_THRESHOLD=-70

while [[ true ]]; do

    IW_OUT=$(iwconfig 2>/dev/null $INTERFACE)
    CURRENT_AP=$(echo "$IW_OUT" | grep "Access Point" | sed -E 's/.*Point: (.*)/\1/' | grep -oE '([0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2}')
    DB=$(echo $IW_OUT | grep level | sed -E 's/.*Signal level=(-[0-9]+).*/\1/')
    echo "Currently connected to $CURRENT_AP with strenght $DB db"

    if [[ $DB -le $DETECTION_THRESHOLD ]]
    then

        BEST_AP=$(sudo wpa_cli -i scan_results | grep -w $ESSID | sort -nrk3 | awk -v th="$DETECTION_THRESHOLD" '$3>=th {print $1}' | head -n1)
        if [[ x"$BEST_AP" == "x" ]] || [[ "${BEST_AP^^}" == "${CURRENT_AP^^}" ]]
        then
            # Since there is no better AP to connect to, we scan for new APs
            echo "There is still no AP guaranteeing a signal > ${DETECTION_THRESHOLD} db"
            RET=$(sudo wpa_cli -i scan freq=${FREQ})
            echo $RET
        else
            # We see a better AP to connect to
            echo "Best AP is $BEST_AP. Roaming"
            sudo wpa_cli roam $BEST_AP
            sleep $SLEEP_DURATION # Arbitrary value to wait for the connection to be established without triggering a scan
        fi
    fi

    sleep $SLEEP_DURATION

done