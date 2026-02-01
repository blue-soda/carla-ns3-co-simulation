#!/bin/bash
set -e

# Download ns-3 with extensions for V2X:

if [ ! -d "ns-3-dev" ]; then
    echo "Cloning ns-3-dev repository..."
    git clone https://gitlab.com/cttc-lena/ns-3-dev.git
else
    echo "Directory 'ns-3-dev' already exists. Skipping clone."
fi

# Switch to the ns-3 development branch with V2X extensions:

cd ns-3-dev

if git show-ref --verify --quiet refs/heads/v2x-lte-dev; then
    echo "Branch 'v2x-lte-dev' already exists. Checking it out..."
    git checkout v2x-lte-dev
else
    echo "Creating and checking out branch 'v2x-lte-dev' from origin..."
    git checkout -b v2x-lte-dev origin/v2x-lte-dev
fi

# Download nr with extensions for V2X:

cd contrib

if [ ! -d "nr" ]; then
    echo "Cloning NR repository with V2X extensions..."
    git clone https://gitlab.com/cttc-lena/nr.git
else
    echo "Directory 'nr' already exists. Skipping clone."
fi
# Switch to the development branch with V2X extensions:

cd nr

if git show-ref --verify --quiet refs/heads/nr-v2x-dev; then
    echo "Branch 'nr-v2x-dev' already exists. Checking it out..."
    git checkout nr-v2x-dev
else
    echo "Creating and checking out branch 'nr-v2x-dev' from origin..."
    git checkout -b nr-v2x-dev origin/nr-v2x-dev
fi

echo "[INFO] ns-3 downloaded and unpacked"

# Copy bridge code to ns-3

cd ../../..

pwd

ln -sf $(pwd)/ns3/vanet/ $(pwd)/ns-3-dev/scratch/
ln -sf $(pwd)/ns3/src/* $(pwd)/ns-3-dev/contrib/nr/model/
ln -sf $(pwd)/ns3/cmake/* $(pwd)/ns-3-dev/contrib/nr/

echo "[INFO] copied bridge code to ns-3"

# Configuring and building NR V2X extensions

cd ns-3-dev

./ns3 configure --enable-tests --enable-examples

./ns3 build

echo "[INFO] ns-3 installation completed"