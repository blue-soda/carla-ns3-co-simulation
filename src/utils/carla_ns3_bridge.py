import json
import socket
import threading
import time
from typing import Dict, Any, List, Callable, Optional
from src.utils.logger import logger
from config.settings import SOCKET_PORT

class CarlaNs3Bridge:
    """Bridge for communication between CARLA and ns-3 using standard sockets"""
    
    def __init__(self, ns3_host: str = 'localhost', ns3_port: int = SOCKET_PORT):
        self.ns3_host = ns3_host
        self.ns3_port = ns3_port
        self.socket = None
        self.connected = False
        self.running = True
        self.reconnect_thread = None
        self.receiver_thread = None
        self.received_messages = []
    
    def _connect(self) -> bool:
        """Connect to ns-3 server"""
        if self.socket:
            self.socket.close()
            
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((self.ns3_host, self.ns3_port))
            self.connected = True
            logger.info(f"Connected to ns-3 bridge at {self.ns3_host}:{self.ns3_port}")
            return True
        except Exception as e:
            logger.error(f"Error connecting to ns-3 bridge: {e}")
            self.connected = False
            return False
    
    def _reconnect_loop(self):
        """Try to reconnect periodically"""
        while self.running and not self.connected:
            if self._connect():
                break
            time.sleep(5)
    
    def ensure_connection(self):
        """Ensure there's a connection to ns-3, try to reconnect if not"""
        if not self.connected and not self.reconnect_thread:
            self.reconnect_thread = threading.Thread(target=self._reconnect_loop)
            self.reconnect_thread.daemon = True
            self.reconnect_thread.start()
    
    def _receive_loop(self, callback: Callable[[Dict[str, Any]], None] = None):
        """Loop to receive messages from ns-3"""
        buffer_size = 4096
        
        while self.running and self.connected:
            try:
                self.socket.settimeout(0.5)
                
                try:
                    data = self.socket.recv(buffer_size)
                    if not data:
                        logger.info("Connection closed by ns-3")
                        self.connected = False
                        self.ensure_connection()
                        break
                    
                    message = data.decode('utf-8')
                    if message:
                        try:
                            json_data = json.loads(message)
                            self.received_messages.append(json_data)
                            if callback:
                                callback(json_data)
                        except json.JSONDecodeError as e:
                            logger.error(f"Error decoding JSON from ns-3: {e}")
                except socket.timeout:
                    pass
                except socket.error as e:
                    logger.error(f"Socket error while receiving data: {e}")
                    self.connected = False
                    self.ensure_connection()
                    break
                    
            except Exception as e:
                logger.error(f"Error in receive loop: {e}")
                time.sleep(1)
    
    def start_receiver(self, callback: Callable[[Dict[str, Any]], None] = None):
        """Start a thread to receive messages from ns-3"""
        if not self.receiver_thread:
            self.receiver_thread = threading.Thread(target=self._receive_loop, args=(callback,))
            self.receiver_thread.daemon = True
            self.receiver_thread.start()
            logger.info("Started receiver thread for messages from ns-3")
    
    def send_vehicle_states(self, vehicles):
        """Send vehicle states to ns-3"""
        if not self.connected:
            logger.warning("Not connected, attempting to reconnect...")
            self.ensure_connection()
            if not self.connected:
                logger.error("Failed to reconnect")
                return False
        
        try:
            message = json.dumps(vehicles)
            self.socket.sendall((message + "\n").encode('utf-8'))
            logger.info(f"Sent {len(message)} bytes to NS-3 successfully")
            return True
        except Exception as e:
            logger.error(f"Error sending vehicle states: {e}")
            self.connected = False
            return False
    
    def send_v2x_message(self, message: Dict[str, Any]) -> bool:
        """Send a V2X message to ns-3"""
        if not self.connected:
            self.ensure_connection()
            return False
        
        try:
            data = {
                "type": "v2x_message",
                "message": message
            }
            msg = json.dumps(data)
            self.socket.sendall(msg.encode('utf-8'))
            return True
        except Exception as e:
            logger.error(f"Error sending V2X message: {e}")
            self.connected = False
            self.ensure_connection()
            return False
    
    def stop(self):
        """Stop the bridge"""
        self.running = False
        if self.socket:
            self.socket.close()
        if self.reconnect_thread:
            self.reconnect_thread.join(timeout=1.0)
        if self.receiver_thread:
            self.receiver_thread.join(timeout=1.0)