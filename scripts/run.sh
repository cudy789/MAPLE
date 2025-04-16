#!/bin/bash

IMAGE_NAME="maple"
IMAGE_TAG="latest"

if [ "$ARM" = "1" ] || [ "$( uname -m )" = "aarch64" ]; then
  ARCH="--platform=linux/arm64"
else
  ARCH="--platform=linux/amd64"
  IMAGE_TAG="X64"
fi

# Start this docker container from the specified image, adding a unique hostname and proper networking
# Also forwards any GUI applications to the host and also adds the current user to the container and mounts its home directory

set -e # Exit on error

while true; do

  echo "Using image rogueraptor7/$IMAGE_NAME:$IMAGE_TAG"

  docker run --rm -h $IMAGE_NAME-$HOSTNAME --name maple-container --group-add sudo --group-add video --add-host $IMAGE_NAME-$HOSTNAME:127.0.0.1 --network host -it \
    --user=$(id -u $USER):$(id -g $USER) \
    --volume="/etc/passwd:/etc/passwd:ro" \
    --volume="/etc/shadow:/etc/shadow:ro" \
    --volume=".:/app" \
    --workdir="/app" \
    --privileged \
    --pids-limit=-1 \
    $ARCH \
    rogueraptor7/$IMAGE_NAME:$IMAGE_TAG /bin/bash -c "cd bin && ./maple"

  done