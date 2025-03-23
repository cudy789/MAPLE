#!/bin/bash

# Corey Knutson, 1/29/2025

# Create simulation videos with specific trajectories and camera parameters specified in the sim_configs folder.
# Run this script from the root directory of MAPLE

IMAGE_NAME="maple"
IMAGE_TAG="latest"

if [ "$ARM" = "1" ] || [ "$( uname -m )" = "aarch64" ]; then
  ARCH="--platform=linux/arm64"
  if [ "$( uname -m )" != "aarch64" ]; then
    docker run --rm --privileged multiarch/qemu-user-static --reset -p yes # enable qemu support for docker
  fi
else
  ARCH="--platform=linux/amd64"
  IMAGE_TAG="X64"
fi

echo "Generate simulation videos for integration tests"

echo "Pulling latest image for simulation testing: rogueraptor7/$IMAGE_NAME:$IMAGE_TAG"
docker pull rogueraptor7/$IMAGE_NAME:$IMAGE_TAG

cd tools;
NUM_FILES=$(ls ../test/integration/sim_tests/sim_configs/*.yml -1 | wc -l)
FILE_COUNT=1
set -e  # exit on error
for t in ../test/integration/sim_tests/sim_configs/*.yml; do
#for t in ../test/integration/sim_tests/sim_configs/2025-triple-cam-pathplanner.yml; do
 # Generate videos
  echo "Kill any existing maple-integration containers..."
  docker kill maple-integration || true
  sleep 2
  echo "Generating videos for test $FILE_COUNT / $NUM_FILES: $t"
  docker run --rm -h $IMAGE_NAME-$HOSTNAME --name maple-integration --group-add sudo --group-add video --add-host $IMAGE_NAME-$HOSTNAME:127.0.0.1 --network host \
    --user=$(id -u $USER):$(id -g $USER) \
    --volume="/etc/passwd:/etc/passwd:ro" \
    --volume="/etc/shadow:/etc/shadow:ro" \
    --volume="$(pwd)/..:$(pwd)/.." \
    --workdir="$(pwd)" \
    --privileged \
    $ARCH \
    rogueraptor7/$IMAGE_NAME:$IMAGE_TAG /bin/bash -c "python3 pybullet_video_gen.py --config \"$t\" --output ../test/integration/sim_tests/sim_output"

  FILE_COUNT=$((FILE_COUNT + 1))
done