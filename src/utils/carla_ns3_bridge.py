import zmq
import json
import threading
from typing import Dict, Any, List

class CarlaNs3Bridge:
    """Bridge for communication between CARLA and ns-3"""
    
    def __init__(self, send_port: int = 5556, recv_port: int = 5557):
        # Setup ZMQ context
        self.context = zmq.Context()
        
        # Socket to send data to ns-3
        self.sender = self.context.socket(zmq.PUB)
        self.sender.bind(f"tcp://*:{send_port}")
        
        # Socket to receive data from ns-3
        self.receiver = self.context.socket(zmq.SUB)
        self.receiver.connect(f"tcp://localhost:{recv_port}")
        self.receiver.setsockopt(zmq.SUBSCRIBE, b"")
        
        # Flag to control receiver thread
        self.running = True
        self.receiver_thread = None
        self.received_messages = []
        
    def start_receiver(self, callback=None):
        """Start a thread to receive messages from ns-3"""
        def receive_loop():
            while self.running:
                try:
                    message = self.receiver.recv_string(flags=zmq.NOBLOCK)
                    data = json.loads(message)
                    self.received_messages.append(data)
                    if callback:
                        callback(data)
                except zmq.Again:
                    # No message available
                    pass
                except Exception as e:
                    print(f"Error receiving message: {e}")
        
        self.receiver_thread = threading.Thread(target=receive_loop)
        self.receiver_thread.daemon = True
        self.receiver_thread.start()
        
    def send_vehicle_states(self, vehicles: List[Dict[str, Any]]):
        """Send vehicle states to ns-3"""
        try:
            message = json.dumps(vehicles)
            self.sender.send_string(message)
        except Exception as e:
            print(f"Error sending vehicle states: {e}")
    
    def send_v2x_message(self, message: Dict[str, Any]):
        """Send a V2X message to ns-3"""
        try:
            data = {
                "type": "v2x_message",
                "message": message
            }
            self.sender.send_string(json.dumps(data))
        except Exception as e:
            print(f"Error sending V2X message: {e}")
            
    def stop(self):
        """Stop the bridge"""
        self.running = False
        if self.receiver_thread:
            self.receiver_thread.join(timeout=1.0)
        self.sender.close()
        self.receiver.close()
        self.context.term()