#!/bin/bash
set -e

# Install dependencies
sudo apt-get update
sudo apt-get install -y build-essential autoconf automake libboost-all-dev libzmq3-dev \
                        git gcc g++ python3 python3-dev pkg-config sqlite3 \
                        libsqlite3-dev cmake libc6-dev libc6-dev-i386 libclang-dev \
                        nlohmann-json3-dev

# Clone ns-3
if [ ! -d "ns-3" ]; then
    git clone --branch ns-3.35 https://gitlab.com/nsnam/ns-3-dev.git ns-3
    cd ns-3
    
    # Configure ns-3
    ./waf configure --enable-examples --enable-tests
    
    # Build ns-3
    ./waf build
    
    echo "ns-3 installation completed"
else
    echo "ns-3 directory already exists, skipping installation"
fi

# Copy our bridge code to ns-3
sudo cp ../ns3/carla-v2x-bridge.cc ./ns-3/scratch/

echo "Setup completed successfully!"