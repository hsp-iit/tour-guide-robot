#!/bin/bash

# Change: we want this to work with both 192.168.100 and 192.168.101
current_ip=$(ifconfig | grep 192.168.10 | awk '{print $2}')

echo $current_ip

case "$current_ip" in
  192.168.101.10)
    echo "base"
    sed -i -e "s/REMOTEIP/192\.168\.101\.1/g" ${TOUR_GUIDE_ROBOT_SOURCE_DIR}/app/navigation2/conf/cyclone_dds_settings.xml
    ;;
  192.168.100.10)
    echo "base"
    sed -i -e "s/REMOTEIP/192\.168\.100\.1/g" ${TOUR_GUIDE_ROBOT_SOURCE_DIR}/app/navigation2/conf/cyclone_dds_settings.xml
    ;;
  192.168.101.*)
    echo "base"
    sed -i -e "s/REMOTEIP/192\.168\.101\.10/g" ${TOUR_GUIDE_ROBOT_SOURCE_DIR}/app/navigation2/conf/cyclone_dds_settings.xml
    ;;
  192.168.100.*)
    echo "base"
    sed -i -e "s/REMOTEIP/192\.168\.100\.10/g" ${TOUR_GUIDE_ROBOT_SOURCE_DIR}/app/navigation2/conf/cyclone_dds_settings.xml
    ;;
  *)
    echo "Unknown ip address"
    ;;
esac

sed -i -e "s/LOCALIP/$current_ip/g" ${TOUR_GUIDE_ROBOT_SOURCE_DIR}/app/navigation2/conf/cyclone_dds_settings.xml

