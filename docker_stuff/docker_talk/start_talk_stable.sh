xhost + local:docker

docker compose --env-file env_config/stable.env up r1-talk-container  --attach r1-talk-cuda-build