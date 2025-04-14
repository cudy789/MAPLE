#!/bin/bash

# Download and extract base image
echo "Downloading latest Raspberry Pi OS arm64 image"
wget https://downloads.raspberrypi.com/raspios_lite_arm64_latest -O raspbian.img.xz
echo "Extracting image"
xz -dk raspbian.img.xz
#IMG=$(ls *raspios-lite-arm64-*.img)
IMG=raspbian.img

truncate -s +10G $IMG  # Add 10GB to image size

# Setup loop device
LOOP_DEV=$(sudo losetup -Pf --show "$IMG")

# Specify cleanups during exit
cleanup() {
    echo "Cleaning up loop devices and mounts..."
    sudo umount -lf /mnt/pi-root/dev/pts || true
    sudo umount -lf /mnt/pi-root/var/run/docker.sock || true
    sudo umount -lf /mnt/pi-root/run || true
    sudo umount -lf /mnt/pi-root/{sys,proc,dev} || true
    sudo umount -lf /mnt/pi-root || true
    if [ -n "$LOOP_DEV" ]; then
        sudo losetup -d "$LOOP_DEV" || true
    fi
}
trap cleanup EXIT

ROOT_PART="${LOOP_DEV}p2"

sudo losetup -Pf $IMG
sudo parted "$LOOP_DEV" resizepart 2 100%

# Mount root partition
echo "Create and mount partitions"
sudo mkdir -p /mnt/pi-root
sudo systemctl daemon-reexec
sudo mount "$ROOT_PART" /mnt/pi-root

# Resize partition
sudo resize2fs "$ROOT_PART"

# Prepare chroot environment
sudo mount --bind /dev /mnt/pi-root/dev
sudo mount --bind /proc /mnt/pi-root/proc
sudo mount --bind /sys /mnt/pi-root/sys
sudo mount --bind /dev/pts /mnt/pi-root/dev/pts
sudo mount --bind /var/run/docker.sock /mnt/pi-root/var/run/docker.sock
sudo mount --bind /run /mnt/pi-root/run  # Required for systemd services

# Configure the image
echo "Configure the image"
echo '127.0.0.1 localhost' | sudo tee /mnt/pi-root/etc/hosts; echo '127.0.1.1 maple' | sudo tee -a /mnt/pi-root/etc/hosts;
echo 'maple' | sudo tee /mnt/pi-root/etc/hostname;

sudo chroot /mnt/pi-root /bin/bash -c 'echo "pi:raspberry" | chpasswd'  # Set default password
sudo chroot /mnt/pi-root /bin/bash -c "usermod -aG sudo pi"
sudo chroot /mnt/pi-root /bin/bash -c "systemctl enable ssh" # Enable ssh server
sudo cp ../../install.sh /mnt/pi-root/install.sh
sudo chroot /mnt/pi-root /bin/bash -c "chmod +x /install.sh && SKIP_STATIC_IP=1 bash /install.sh"
sudo chroot /mnt/pi-root /bin/bash -c "usermod -aG docker pi"


# Point Docker storage to inside the mounted image
sudo echo '{ "data-root": "/mnt/pi-root/var/lib/docker" }' | sudo tee /etc/docker/daemon.json
echo "Changed Docker data root directory, restarting Docker..."
sudo systemctl restart docker

sudo chroot /mnt/pi-root /bin/bash -c "cd maple-config && docker-compose pull"
sudo cp maple-docker-compose-init.service /mnt/pi-root/etc/systemd/system/maple-docker-compose-init.service
sudo chroot /mnt/pi-root systemctl enable maple-docker-compose-init.service

# Cleanup
echo "Cleanup"
# Reset the Docker storage location by removing the configuration
sudo rm /etc/docker/daemon.json
echo "Changed Docker data root directory, restarting Docker..."
sudo systemctl restart docker
# Unmount partitions
sudo umount /mnt/pi-root/dev/pts || true
sudo umount /mnt/pi-root/var/run/docker.sock || true
sudo umount /mnt/pi-root/run || true
sudo umount /mnt/pi-root/{sys,proc,dev} || true
sudo umount /mnt/pi-root || true
sudo losetup -d "$LOOP_DEV" || true

# Shrink image
echo "Shrink image"
git clone https://github.com/Drewsif/PiShrink.git
sudo ./PiShrink/pishrink.sh "$IMG" maple.img

## Compress image
#echo "Compress image"
#gzip -c maple.img > maple.img.gz
#
#echo "Raspberry Pi image created: maple.img.gz"
