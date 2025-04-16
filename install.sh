#!/bin/bash

echo "Installing Docker and docker-compose"

# Update package index
sudo apt update

# Install Docker Engine and Docker Compose
sudo apt install -y docker.io

# Enable and start Docker service
sudo systemctl enable docker
sudo systemctl start docker

# Add user to docker group
sudo usermod -aG docker "$USER"

# Make config directory
mkdir -p ./maple
cd ./maple

# Get config files
wget https://raw.githubusercontent.com/cudy789/MAPLE/refs/heads/main/config.yml
wget https://raw.githubusercontent.com/cudy789/MAPLE/refs/heads/main/scripts/run.sh

# Get the latest .syrup file
wget https://raw.githubusercontent.com/cudy789/MAPLE/refs/heads/main/release/arm/latest.syrup
unzip latest.syrup -d ./bin
rm latest.syrup

# Setup the static IP
wget https://raw.githubusercontent.com/cudy789/MAPLE/refs/heads/main/set-ip.sh

if [[ -z "$SKIP_STATIC_IP" ]]; then
  sudo bash -i set-ip.sh
else
  echo "Skipping static IP configuration"
fi

echo "############ Installation is almost complete! ############

To finish the installation:

1. Log out of your terminal and log back in

Then, to configure and start MAPLE:

1. Modify your maple-config/config.yml file for your specific camera setup
2. Run 'docker-compose up -d' to start MAPLE

##########################################################
"
