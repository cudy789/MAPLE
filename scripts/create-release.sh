#!/bin/bash

# Determine RELEASE_VERSION based on CLI argument or default
if [ $# -ge 1 ]; then
    RELEASE_VERSION="$1"
else
    RELEASE_VERSION="MAPLE-R25-0-1"
fi

IMAGE_NAME="maple"
IMAGE_TAG="latest"

if [ "$ARM" = "1" ] || [ "$( uname -m )" = "aarch64" ]; then
  ARCH="arm64"
  ARCH_D="--platform=linux/arm64"
else
  ARCH="x86"
  ARCH_D="--platform=linux/amd64"
  IMAGE_TAG="X64"
fi
mkdir -p release/"$ARCH"

# Build MAPLE
rm -rf build

docker run --rm -h maple --name maple --group-add sudo --group-add video --add-host $IMAGE_NAME-$HOSTNAME:127.0.0.1 --network host \
  --user=$(id -u $USER):$(id -g $USER) \
  --volume="/etc/passwd:/etc/passwd:ro" \
  --volume="/etc/shadow:/etc/shadow:ro" \
  --volume="$HOME:$HOME" \
  --workdir="$(pwd)" \
  --privileged \
  --pids-limit=-1 \
  $ARCH_D \
  rogueraptor7/$IMAGE_NAME:$IMAGE_TAG /bin/bash -c "mkdir -p build && cd build && cmake .. && make -j4"

# Make sure there aren't any logfiles
rm -rf build/logs/*

# Copy over the latest .fmap file
cp fmap/field.fmap build

# Check if a file with the name RELEASE_VERSION exists, and delete it
if [ -f "$RELEASE_VERSION".txt ]; then
    echo "File '$RELEASE_VERSION.txt' exists. Deleting..."
    rm "$RELEASE_VERSION".txt
fi

# Generate the release version file
echo $RELEASE_VERSION > build/version.txt

# Create the .syrup release file
cd build && zip -r ../release/"$ARCH"/"$RELEASE_VERSION"_"$ARCH".syrup .