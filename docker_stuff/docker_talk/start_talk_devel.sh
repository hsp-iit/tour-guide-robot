xhost + local:docker

docker compose --env-file env_config/devel.env up r1-talk-container --attach r1-talk-cuda-build