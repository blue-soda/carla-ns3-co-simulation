# System Requirements

Before starting the installation, please ensure your system meets the following requirements.

- **Operating System:** Ubuntu 22.04 LTS
- **GPU:** Minimum: 6 GB VRAM. Recommended: 8+ GB VRAM.
- **Disk Space:** Minimum 25 GB free.
- **Python:** 3.11 (with pip3).
- **Shell:** Bash or Zsh.
- **Git:** Required.
- **Internet Connection:** Required.

**Important:** Ensure your graphics drivers are up-to-date and properly configured, as CARLA heavily relies on the GPU for rendering.

# Installation Guide

CARLA-NS3-co-simulation consists of three main components:

1.  **CARLA:** An autonomous driving simulator that provides a Python API for controlling vehicles and interacting with the virtual environment.
2.  **NS3:** A network simulator designed for modeling network interactions.
3.  **CARLA-NS3-co-simulation:** An interface that enables interaction and synchronization between CARLA and NS3.

## Step 1: Install CARLA

The first step is to install the CARLA simulator. This guide requires CARLA version 0.9.15. If you already have this version installed, you can skip this step. If you have a different version, you may need to reinstall CARLA or ensure compatibility with the co-simulation interface.

Detailed installation instructions for CARLA are available in the official documentation: [CARLA 0.9.15 Quick Start](https://carla.readthedocs.io/en/0.9.15/start_quickstart/#carla-installation).

Ensure you have installed all necessary dependencies and successfully built CARLA according to the official documentation. You can install CARLA in any directory, but using `~/carla` is recommended for convenience.

**Example of launching CARLA:**

After successful installation, navigate to the CARLA directory and execute the command:

```bash
./CarlaUE4.sh
```

## Step 2: Install NS3 with CARLA-NS3-co-simulation modules

After installing and verifying CARLA, you can proceed to set up the co-simulation interface and NS3.

1.  **Clone the CARLA-NS3-co-simulation repository:**

    ```bash
    git clone git@github.com:TechOctopus/carla-ns3-co-simulation.git
    ```

2.  **Navigate into the project directory:**

    ```bash
    cd carla-ns3-co-simulation
    ```

3.  **Install system dependencies for NS3:**

    - This step is typically performed **only once** when first installing NS3 on your system.

    ```bash
    sudo ./installdependencies.sh
    ```

4.  **Install NS3 and required modules:**
    - This script will download and build NS3, and integrate the necessary components for co-simulation. This process may take some time.
    ```bash
    ./install_ns3.sh
    ```
    Upon completion, you should see messages confirming successful execution:
    ```bash
    ...
    "[INFO] ns-3 downloaded and unpacked"
    ...
    "[INFO] copied bridge code to ns-3"
    ...
    "[INFO] ns-3 installation completed"
    ```

## Step 3: Set up the Python Environment

To run the co-simulation scripts, you need to set up an isolated Python environment and install the dependencies. The guide use virtual environments to manage Python packages but you can also use Anaconda or other package managers if preferred.

5.  **Create a Python virtual environment:**

    - Using a virtual environment is recommended to isolate project dependencies.

    ```bash
    python3 -m venv carla
    ```

6.  **Activate the virtual environment:**

    - After activation, your command prompt should change to indicate the active environment (`(carla)`).

    ```bash
    source carla/bin/activate
    ```

7.  **Install Python dependencies:**
    ```bash
    pip install -r requirements.txt
    ```

## Installation Complete

After completing all steps, the CARLA-NS3-co-simulation system is ready for use. You have installed CARLA, NS3 with the necessary modules, and configured the Python environment with all dependencies.

# Run the co-simulation

To run the CARLA-NS3 co-simulation, you need to follow these steps in order:

1.  **Launch CARLA Simulator:**

    - Open a terminal and navigate to your CARLA installation directory:
    ```bash
    cd ~/<path-to-carla-installation>

    ./CarlaUE4.sh
    ```

2.  **Start NS3 VANET Simulation:**

    - Open a new terminal and navigate to the ns-3 directory in the main project directory:
    ```bash
    cd ns-allinone-3.44/ns-3.44
    ```
    
    - Run the VANET simulation with desired parameters:
    ```bash
     ./ns3 run scratch/vanet/main.cc -- --simTime=300.0
    ```
    
    Parameters:
    - `simTime`: Simulation duration in seconds (default: 10.0)

3.  **Run the CARLA-NS3 Bridge:**

    - Open a new terminal and navigate to the project directory. Then activate the Python virtual environment:
    ```bash
    source carla/bin/activate
    ```
    
    - Run the main simulation script:
    ```bash
    python main.py
    ```

4.  **Monitor the Simulation:**

    - The simulation progress can be monitored through:
        - CARLA visualization window
        - Terminal outputs showing vehicle states and network messages
        - Log files in the `temp` directory
    
    - After simulation completion, visualization plots will be automatically generated in the `temp` directory:
        - Vehicle trajectories
        - Speed over time
        - Heading over time

5.  **Stop the Simulation:**

    - The simulation can be stopped by:
        - Letting it run for the specified simulation time
        - Pressing Ctrl+C in the Python script terminal, ns3 terminal and carla terminal
    
    - The system will automatically:
        - Stop the NS3 simulation
        - Clean up CARLA actors
        - Generate final visualization plots
        - Save all logs including PCAP files

**Important Notes:**

- Ensure all three components (CARLA, NS3, and the bridge) are running simultaneously
- The default map is set to 'Town10HD_Opt' but can be changed in `config/settings.py`
- Vehicle data is logged to `temp/vehicle_data.json`
- Visualization plots are saved in PDF format in the `temp/plots` directory
- Camera data is saved in `temp/camera` directory
- For debugging, check the log files in the `temp` directory
