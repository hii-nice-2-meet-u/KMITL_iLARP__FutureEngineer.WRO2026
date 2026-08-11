#!/bin/bash
set -e

PI_USER="hii"
PI_IP="192.168.2.107"
PI_TARGET_DIR="/home/hii/wro2026_workspace"

if [ "$1" == "clean" ]; then
    echo "Cleaning local dist artifacts..."
    rm -rf dist
    echo "Clean finished."
    exit 0
fi

TARGET="$1"

echo "Building via Docker Cross-Compiler..."
docker build -f code/Dockerfile.compile -t wro2026-app-build code/

echo "Extracting compiled artifacts..."
rm -rf dist && mkdir -p dist
docker create --name temp_export wro2026-app-build /bin/true
docker cp temp_export:/install ./dist/
docker rm temp_export

echo "Deploying to Raspberry Pi 5 ($PI_IP)..."
ssh $PI_USER@$PI_IP "mkdir -p $PI_TARGET_DIR"
# rsync -avz --delete --exclude 'include/' ./dist/install/ $PI_USER@$PI_IP:$PI_TARGET_DIR/
rsync -avz --delete  ./dist/install/ $PI_USER@$PI_IP:$PI_TARGET_DIR/

ssh "$PI_USER@$PI_IP" \
    "find $PI_TARGET_DIR/app -type f -exec chmod +x {} +"
echo "SUCCESS! Deployed to Pi 5 at: $PI_TARGET_DIR"