import math
import json
import datetime
import random
from typing import Dict, Any

class CAMGenerator:
    """
    Cooperative Awareness Message generator
    """
    def __init__(self, vehicle):
        self.vehicle = vehicle
        
    def generate_message(self) -> Dict[str, Any]:
        """Generate a CAM message based on current vehicle state"""
        # Get vehicle transform and velocity
        transform = self.vehicle.get_transform()
        velocity = self.vehicle.get_velocity()
        
        # Calculate speed (in km/h)
        speed = 3.6 * math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
        
        # Create CAM message
        cam_message = {
            "messageID": "CAM",
            "stationID": self.vehicle.id,
            "timestamp": datetime.datetime.now().isoformat(),
            "position": {
                "latitude": transform.location.y,
                "longitude": transform.location.x,
                "altitude": transform.location.z
            },
            "speed": speed,
            "heading": transform.rotation.yaw,
            "acceleration": {
                "x": self.vehicle.get_acceleration().x,
                "y": self.vehicle.get_acceleration().y,
                "z": self.vehicle.get_acceleration().z
            }
        }
        
        return cam_message
        
    def broadcast(self):
        """Broadcast the CAM message"""
        message = self.generate_message()

        print(f"Broadcasting CAM: {json.dumps(message, indent=2)}")
        return message

class DENMGenerator:
    """
    Decentralized Environmental Notification Message generator
    """

    def __init__(self, vehicle):
        self.vehicle = vehicle
    

    def generate_message(self, event_type, description, severity="medium") -> Dict[str, Any]:
        """Create a DENM message"""
        
        # Get vehicle location
        location = self.vehicle.get_location()
        
        denm_message = {
            "messageID": "DENM",
            "eventID": random.randint(1, 10000),
            "timestamp": datetime.datetime.now().isoformat(),
            "eventType": event_type,
            "eventPosition": {
                "latitude": location.y,
                "longitude": location.x,
                "altitude": location.z
            },
            "eventDescription": description,
            "eventSeverity": severity,
            "validityDuration": 60  # seconds
        }
        
        return denm_message
        
    @staticmethod
    def broadcast(message):
        """Broadcast the DENM message"""
        
        print(f"Broadcasting DENM: {json.dumps(message, indent=2)}")
        return message
