#!/bin/bash
set -e

# Cheak is sudo or not
if [ "$EUID" -ne 0 ]
  then echo "[ERROR] Please run with sudo privileges"
  exit
fi

# Install dependencies
sudo apt-get update
sudo apt-get install -y build-essential autoconf automake libboost-all-dev libzmq3-dev \
                        git gcc g++ python3 python3-dev pkg-config sqlite3 \
                        libsqlite3-dev cmake libc6-dev libc6-dev-i386 libclang-dev \
                        nlohmann-json3-dev

echo "[INFO] Dependencies installation completed"