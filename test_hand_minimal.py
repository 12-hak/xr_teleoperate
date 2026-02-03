#!/usr/bin/env python3
"""Minimal test to verify hand commands work exactly like the example"""

import time
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from inspire_sdkpy import inspire_hand_defaut, inspire_dds

if __name__ == '__main__':
    print("Initializing DDS...")
    ChannelFactoryInitialize(0)
    
    print("Creating publishers...")
    pubr = ChannelPublisher("rt/inspire_hand/ctrl/r", inspire_dds.inspire_hand_ctrl)
    pubr.Init()
    
    publ = ChannelPublisher("rt/inspire_hand/ctrl/l", inspire_dds.inspire_hand_ctrl)
    publ.Init()
    
    print("Getting command template...")
    cmd = inspire_hand_defaut.get_inspire_hand_ctrl()
    
    print("\nTest 1: Send [500, 500, 500, 500, 500, 500] (mid position)")
    cmd.angle_set = [500, 500, 500, 500, 500, 500]
    cmd.mode = 0b0001
    
    if publ.Write(cmd) and pubr.Write(cmd):
        print("  ✓ Commands sent successfully")
    else:
        print("  ✗ Failed to send commands")
    
    time.sleep(2.0)
    
    print("\nTest 2: Send [200, 200, 200, 200, 200, 200] (more closed)")
    cmd.angle_set = [200, 200, 200, 200, 200, 200]
    cmd.mode = 0b0001
    
    if publ.Write(cmd) and pubr.Write(cmd):
        print("  ✓ Commands sent successfully")
    else:
        print("  ✗ Failed to send commands")
    
    time.sleep(2.0)
    
    print("\nTest 3: Send [800, 800, 800, 800, 800, 800] (more open)")
    cmd.angle_set = [800, 800, 800, 800, 800, 800]
    cmd.mode = 0b0001
    
    if publ.Write(cmd) and pubr.Write(cmd):
        print("  ✓ Commands sent successfully")
    else:
        print("  ✗ Failed to send commands")
    
    time.sleep(2.0)
    
    print("\n✓ Test complete. Did the hands move?")
