import os
import numpy as np
from PIL import Image
from io import BytesIO

def save_map_image(map_name: str = "town_map"):
    """Save the map image to the maps directory
    
    Args:
        map_name: Name of the map file without extension
    """
    # Get the directory where this file is located
    current_dir = os.path.dirname(os.path.abspath(__file__))
    # Create maps directory if it doesn't exist
    maps_dir = os.path.join(current_dir, '..', '..', 'maps')
    os.makedirs(maps_dir, exist_ok=True)
    
    map_path = os.path.join(maps_dir, f'{map_name}.png')
    
    # If map already exists, don't overwrite it
    if os.path.exists(map_path):
        return map_path
    
    # Create a new map image with the correct dimensions and save it
    # This is a placeholder - you should replace this with your actual map image
    img = Image.new('RGB', (1000, 1000), color='white')
    img.save(map_path)
    
    return map_path

def get_map_bounds(map_name: str = "town_map") -> tuple:
    """Get the coordinate bounds for a specific map
    
    Args:
        map_name: Name of the map
        
    Returns:
        tuple: (min_x, max_x, min_y, max_y) in CARLA coordinates
    """
    # Calibrated bounds based on the trajectory plot
    map_bounds = {
        "town_map": (-100, 100, -100, 100)  # Adjusted bounds for Town10
    }
    
    return map_bounds.get(map_name, (-100, 100, -100, 100))  # Default to Town10 bounds 