import json
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
from typing import List, Dict
import os
from datetime import datetime
from .map_utils import get_map_bounds

class VehicleDataVisualizer:
    def __init__(self, data_file: str):
        """Initialize the visualizer with vehicle data file
        
        Args:
            data_file: Path to the vehicle data JSON file
        """
        self.data_file = data_file
        with open(data_file, 'r') as f:
            self.data = json.load(f)
        
        # Create output directory for plots
        self.output_dir = os.path.join(os.path.dirname(data_file), 'plots')
        os.makedirs(self.output_dir, exist_ok=True)
        
        # Set style for academic plots
        plt.style.use('seaborn-v0_8-paper')
        plt.rcParams.update({
            'font.size': 12,
            'axes.labelsize': 12,
            'axes.titlesize': 14,
            'xtick.labelsize': 10,
            'ytick.labelsize': 10,
            'legend.fontsize': 10,
            'figure.figsize': (12, 12),
            'figure.dpi': 300,
            'savefig.dpi': 300,
            'savefig.format': 'pdf',
            'savefig.bbox': 'tight',
            'savefig.pad_inches': 0.1
        })

    def load_map_image(self):
        """Load and return the map image with its bounds"""
        # Get the directory where visualization.py is located
        current_dir = os.path.dirname(os.path.abspath(__file__))
        maps_dir = os.path.join(current_dir, '..', '..', 'maps')
        map_path = os.path.join(maps_dir, 'town10_map.png')
        
        try:
            map_img = mpimg.imread(map_path)
            bounds = get_map_bounds("town10_map")
            return map_img, bounds
        except Exception as e:
            print(f"Error loading map: {e}")
            return None, (0, 0, 0, 0)

    def plot_vehicle_trajectories(self):
        """Plot vehicle trajectories on the map"""
        fig, ax = plt.subplots(figsize=(12, 12))
        
        # Try to load the map
        map_img, bounds = self.load_map_image()
        if map_img is not None:
            # Display the map
            ax.imshow(map_img, extent=bounds, aspect='equal', zorder=0)
        
        # Get unique vehicle IDs
        vehicle_ids = set()
        for frame in self.data['frames']:
            for vehicle in frame['vehicles']:
                vehicle_ids.add(vehicle['id'])
        
        # Define colors for vehicles
        colors = ['purple', 'lime', 'red', 'blue', 'orange', 'cyan']  # Fixed colors for better visibility
        
        # Plot trajectory for each vehicle
        for vehicle_id, color in zip(vehicle_ids, colors):
            x_coords = []
            y_coords = []
            for frame in self.data['frames']:
                for vehicle in frame['vehicles']:
                    if vehicle['id'] == vehicle_id:
                        x_coords.append(vehicle['position']['x'])
                        y_coords.append(vehicle['position']['y'])
                        break
            
            # Plot the trajectory with higher zorder to appear above the map
            ax.plot(x_coords, y_coords, 
                   label=f'Vehicle {vehicle_id}',
                   color=color,
                   linewidth=2,
                   alpha=0.8,
                   zorder=2)
            
            # Plot start and end points
            if x_coords and y_coords:
                ax.plot(x_coords[0], y_coords[0], 'o', 
                       color=color, 
                       markersize=10,
                       zorder=3)
                ax.plot(x_coords[-1], y_coords[-1], 's', 
                       color=color, 
                       markersize=10,
                       zorder=3)
        
        ax.set_xlabel('X Position (m)')
        ax.set_ylabel('Y Position (m)')
        ax.set_title('Vehicle Trajectories')
        
        # Add a light grid that won't interfere with the map
        ax.grid(True, linestyle='--', alpha=0.3, zorder=1)
        
        # Move legend outside the plot
        ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
        
        # Ensure the aspect ratio is equal
        ax.set_aspect('equal')
        
        # Adjust layout to prevent legend cutoff
        plt.tight_layout()
        
        plt.savefig(os.path.join(self.output_dir, 'trajectories.pdf'), 
                   bbox_inches='tight',
                   pad_inches=0.2)
        plt.close()

    def plot_speed_over_time(self):
        """Plot vehicle speeds over time"""
        fig, ax = plt.subplots()
        
        # Get unique vehicle IDs
        vehicle_ids = set()
        for frame in self.data['frames']:
            for vehicle in frame['vehicles']:
                vehicle_ids.add(vehicle['id'])
        
        # Convert timestamps to relative time in seconds
        start_time = datetime.strptime(self.data['simulation_start'], '%Y%m%d_%H%M%S')
        times = []
        for frame in self.data['frames']:
            frame_time = datetime.strptime(frame['timestamp'], '%Y-%m-%d %H:%M:%S')
            times.append((frame_time - start_time).total_seconds())
        
        # Plot speed for each vehicle
        for vehicle_id in vehicle_ids:
            speeds = []
            for frame in self.data['frames']:
                for vehicle in frame['vehicles']:
                    if vehicle['id'] == vehicle_id:
                        speeds.append(vehicle['speed'])
                        break
            
            ax.plot(times, speeds, label=f'Vehicle {vehicle_id}', linewidth=1.5)
        
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Speed (m/s)')
        ax.set_title('Vehicle Speeds Over Time')
        ax.grid(True, linestyle='--', alpha=0.7)
        ax.legend()
        
        plt.savefig(os.path.join(self.output_dir, 'speeds.pdf'))
        plt.close()

    def plot_heading_over_time(self):
        """Plot vehicle headings over time"""
        fig, ax = plt.subplots()
        
        # Get unique vehicle IDs
        vehicle_ids = set()
        for frame in self.data['frames']:
            for vehicle in frame['vehicles']:
                vehicle_ids.add(vehicle['id'])
        
        # Convert timestamps to relative time in seconds
        start_time = datetime.strptime(self.data['simulation_start'], '%Y%m%d_%H%M%S')
        times = []
        for frame in self.data['frames']:
            frame_time = datetime.strptime(frame['timestamp'], '%Y-%m-%d %H:%M:%S')
            times.append((frame_time - start_time).total_seconds())
        
        # Plot heading for each vehicle
        for vehicle_id in vehicle_ids:
            headings = []
            for frame in self.data['frames']:
                for vehicle in frame['vehicles']:
                    if vehicle['id'] == vehicle_id:
                        headings.append(vehicle['heading'])
                        break
            
            ax.plot(times, headings, label=f'Vehicle {vehicle_id}', linewidth=1.5)
        
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Heading (degrees)')
        ax.set_title('Vehicle Headings Over Time')
        ax.grid(True, linestyle='--', alpha=0.7)
        ax.legend()
        
        plt.savefig(os.path.join(self.output_dir, 'headings.pdf'))
        plt.close()

    def generate_all_plots(self):
        """Generate all available plots"""
        self.plot_vehicle_trajectories()
        self.plot_speed_over_time()
        self.plot_heading_over_time() 