#!/bin/bash

# Corey Knutson, 1/19/2025

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

echo "Pulling latest image for simulation testing: rogueraptor7/$IMAGE_NAME:$IMAGE_TAG"
docker pull rogueraptor7/$IMAGE_NAME:$IMAGE_TAG

BASE_DIR="$(pwd)"

echo "Run simulation integration tests"


NUM_FILES=$(ls test/integration/sim_tests/maple_configs/*.yml -1 | wc -l)
FILE_COUNT=1
set -e  # exit on error
#for t in test/integration/sim_tests/maple_configs/*.yml; do
for t in test/integration/sim_tests/maple_configs/2025-triple-cam-pathplanner.yml; do
#for t in test/integration/sim_tests/maple_configs/2024-dual-cam-center-field-rotation.yml; do

  # Run MAPLE
  IFS="/" read -ra path_array <<< "$t"
  IFS="." read -ra test_name <<< "${path_array[-1]}"

  echo "Running MAPLE integration test $FILE_COUNT / $NUM_FILES: $t"
  echo "Using config file $t"
  echo "Kill any existing maple-integration containers..."
  docker kill maple-integration || true
  sleep 2

  timeout 10m docker run --rm -h $IMAGE_NAME-$HOSTNAME --name maple-integration --group-add sudo --group-add video --add-host $IMAGE_NAME-$HOSTNAME:127.0.0.1 --network host \
    --volume="$BASE_DIR/fmap:/tests/fmap:ro" \
    --volume="$BASE_DIR/build-ci:/tests/build-ci" \
    --volume="$BASE_DIR/include:/tests/include:ro" \
    --volume="$BASE_DIR/lib:/tests/lib:ro" \
    --volume="$BASE_DIR/src:/tests/src:ro" \
    --volume="$BASE_DIR/tools:/tests/tools:ro" \
    --volume="$BASE_DIR/test/integration/sim_tests/sim_output:/tests/test/integration/sim_tests/sim_output" \
    --volume="$BASE_DIR/test/integration/sim_tests/maple_configs/${path_array[-1]}:/tests/config.yml:ro" \
    --volume="$BASE_DIR/CMakeLists.txt:/tests/CMakeLists.txt:ro" \
    --workdir="/tests" \
    --privileged \
    $ARCH \
    rogueraptor7/$IMAGE_NAME:$IMAGE_TAG /bin/bash -c "cat config.yml; mkdir -p build-ci && cd build-ci && rm -rf logs && cmake .. && make -j4 && ./maple 2>&1 | grep -v \"Corrupt JPEG data\"; python3 ../tools/analyze_pose_trajectory.py logs/maple_trajectory_log_* ../test/integration/sim_tests/sim_output/${test_name[0]}_gt.csv"

  FILE_COUNT=$((FILE_COUNT + 1))
done