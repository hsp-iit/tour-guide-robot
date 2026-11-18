#!/bin/bash
cd "$(dirname "$0")"
source env_config/select_dds_conf.sh
xhost + local:docker

docker compose --env-file env_config/stable.env up r1-tourcore-container
