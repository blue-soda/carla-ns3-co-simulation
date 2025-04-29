# Simple script to parse a hex dump 
# of a packet and extract vehicle data

import struct

# Example hex dump of a packet
hex_dump = """
08 00 00 00 ff ff ff ff ff ff 00 00 00 00 00 01
00 00 00 00 00 01 00 00 aa aa 03 00 00 00 00 00
aa aa 03 00 00 00 89 47 01 02 02 c0 48 55 b1 b2
54 a6 76 40 47 bc bb 6d ea 3e 4c 00 00 00 01 03
e8 00 0a 00 00 00 01 c0 48 55 b1 b2 54 a6 76 40
47 bc bb 6d ea 3e 4c 40 00 3d 7d 8d ac e1 65 40
56 6d ef bb 52 cc 23 00 00 00 00 00 00 04 7f 00
00 00 00
"""

packet_bytes = bytes.fromhex(hex_dump.replace("\n", " "))

ether_type_index = packet_bytes.find(b'\x89\x47')
payload = packet_bytes[ether_type_index + 2:]

offset = 4 + 8 + 8 + 4 + 2 + 1

vehicle_id = struct.unpack_from(">I", payload, offset)[0]; offset += 4
position_x = struct.unpack_from(">d", payload, offset)[0]; offset += 8
position_y = struct.unpack_from(">d", payload, offset)[0]; offset += 8
speed = struct.unpack_from(">d", payload, offset)[0]; offset += 8
heading = struct.unpack_from(">d", payload, offset)[0]; offset += 8
timestamp = struct.unpack_from(">Q", payload, offset)[0]; offset += 8

cam_data = {
    "vehicle_id": vehicle_id,
    "position_x": position_x,
    "position_y": position_y,
    "speed": speed,
    "heading": heading,
    "timestamp_ms": timestamp
}

for key, value in cam_data.items():
    print(f"{key}: {value}")
