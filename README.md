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
    git clone https://github.com/blue-soda/carla-ns3-co-simulation.git
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
    - Run the VANET simulation with desired parameters:
    ```bash
    ns-3-dev/ns3 run scratch/vanet/main.cc -- --simTime=100.0
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

# Project Files and Directories

This section provides an overview of the key files and directories within the CARLA-NS3-Co-Simulation project.

# Root Directory Files

## installdependences.sh
This is a shell script responsible for installing all necessary system-level dependencies required to build and run NS3. This script is typically run once during the initial setup of the project on a new system.

## installns3.sh
This shell script automates the process of downloading, building, and configuring NS3. It also integrates the specific modules or patches needed for the co-simulation with CARLA.

## main.py
This is the primary Python script that launches and manages the co-simulation bridge. It orchestrates the interaction between the CARLA simulator and the NS3 network simulator, handling synchronization, data exchange, and overall simulation flow.

## README.md
This file contains essential information about the project. It includes system requirements, detailed installation instructions, steps to run the co-simulation, the project structure, and other relevant notes.

## requirements.txt
This file lists all the Python packages and their specific versions that the project depends on. It is used with `pip install -r requirements.txt` to set up the Python environment correctly, ensuring reproducibility and avoiding dependency conflicts.

# `config` Directory
The `config` directory holds configuration files that allow users to customize various aspects of the simulation without modifying the core source code.

- **`settings.py`**: This Python file contains key-value settings for the simulation. Examples of what might be configured here include the CARLA map to be used (e.g., 'Town10HD_Opt'), default simulation duration, parameters for vehicle behavior.

# `helpers` Directory
This directory contains utility scripts that provide supporting functions for the main application.

- **`decoder.py`**: This Python script provides an example of parsing a hexadecimal dump of a network packet from simulation to extract and display CAM message data such as vehicle ID, position, speed, heading, and timestamp.

# `maps` Directory
The `maps` directory stores image files of CARLA maps. These images are used to plot the vehicle trajectories.

- **`town07_map.png`**: An image file showing the layout of the 'Town07' map from the CARLA simulator.
- **`town10_map.png`**: An image file showing the layout of the 'Town10HD_Opt' map from the CARLA simulator.

# `ns-allinone-3.44` Directory
This directory contains the standard NS-3 (Network Simulator 3) version 3.44. It is automatically downloaded and installed by the `installns3.sh` script directly from the official NS-3 website. This directory includes the complete NS3 framework, libraries, and ancillary tools required to run network simulations.

# `ns3` Directory
This directory houses the NS3-specific components of the co-simulation, including network scenarios and custom communication protocols for VANETs (Vehicular Ad-hoc Networks).

## `vanet` Subdirectory
This subdirectory contains all the C++ source and header files for the NS3 VANET simulation.
- **`cam-application.cc`**: The C++ source file implementing the Cooperative Awareness Message (CAM) application for NS3. This application simulates the generation, transmission, and reception of CAMs by vehicles in the network.
- **`cam-application.h`**: The C++ header file for `cam-application.cc`. It defines the class structure, member variables, and function prototypes for the CAM application.
- **`geo-networking.cc`**: The C++ source file for the GeoNetworking protocol implementation in NS3. GeoNetworking is a network protocol designed for VANETs that uses geographical position information for message routing and dissemination.
- **`geo-networking.h`**: The C++ header file for `geo-networking.cc`. It defines the interfaces, data structures, and constants for the GeoNetworking protocol implementation.
- **`main.cc`**: The main C++ program for the NS3 VANET simulation. This script sets up the network topology (nodes, channels), installs network stacks and applications (like CAM and GeoNetworking) on the nodes, configures mobility models for vehicles, connects the nodes to the CARLA simulator, and starts the NS3 simulation.

# `src` Directory
The `src` directory contains the Python source code for the CARLA-NS3 bridge co-simulation component.

## `bridge` Subdirectory
This subdirectory holds the core logic for the bridge that connects CARLA and NS3.
- **`carla_ns3_bridge.py`**: A Python script that implements the central bridging mechanism. It is responsible for establishing communication channels, synchronizing the two simulators, and relaying data (e.g., vehicle positions from CARLA to NS3, network messages from NS3 back to CARLA).

## `carla` Subdirectory
This subdirectory contains modules specifically for interacting with the CARLA simulator.
- **`carla_connector.py`**: This Python script manages the connection to the CARLA server. It provides functionalities to spawn and control vehicles and sensors in the CARLA environment, retrieve simulation data (like vehicle states, sensor readings), and manage the simulation world.
- **`vehicle_data.py`**: This Python script defines the data structure for vehicle data. It is used to collect vehicle data from CARLA and store returned data in a JSON format.

## `common` Subdirectory
This subdirectory contains common utility modules that are used by various parts of the `src` codebase.
- **`logger.py`**: This Python utility module sets up and provides a centralized logging mechanism. It configures a logger to output messages to both the console and a `simulation.log` file stored in the `temp` directory, offering `info`, `warning`, and `error` level logging methods.
- **`vehicle_data_logger.py`**: This Python script is dedicated to recording vehicle data. It captures frames of vehicle information (ID, position, speed, heading) and saves them incrementally to a `vehicle_data.json` file within the `temp` directory. The log includes the simulation start time and timestamps for each data frame.
- **`visualization.py`**: This Python script uses the `matplotlib` library to generate and save various visual plots from the `vehicle_data.json` file. It creates plots for vehicle trajectories overlaid on a map, vehicle speeds over time, and vehicle headings over time. These visualizations are saved as PDF files in the `temp/plots` directory.

# `temp` Directory
The `temp` directory is designated for storing temporary files generated during or after a simulation run. This includes log files from various components, raw or processed data (like `vehicle_data.json`), visualization plots, and camera sensor data.
