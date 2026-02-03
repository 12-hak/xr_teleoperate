#!/usr/bin/env python3
"""
Exact copy of working dds_publish.py logic to test hands
"""
import time
import numpy as np
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from inspire_sdkpy import inspire_hand_defaut, inspire_dds

if __name__ == '__main__':
    print("Initializing DDS on domain 0...")
    ChannelFactoryInitialize(0)
    
    print("Creating publishers...")
    pubr = ChannelPublisher("rt/inspire_hand/ctrl/r", inspire_dds.inspire_hand_ctrl)
    pubr.Init()
    
    publ = ChannelPublisher("rt/inspire_hand/ctrl/l", inspire_dds.inspire_hand_ctrl)
    publ.Init()
    
    cmd = inspire_hand_defaut.get_inspire_hand_ctrl()
    
    # EXACT initialization from working example
    print("\n=== Initialization sequence (from working example) ===")
    cmd.angle_set = [0, 0, 0, 0, 1000, 1000]
    cmd.mode = 0b0001
    publ.Write(cmd)
    pubr.Write(cmd)
    print("  Sent: [0, 0, 0, 0, 1000, 1000]")
    time.sleep(1.0)
    
    cmd.angle_set = [0, 0, 0, 0, 0, 1000]
    cmd.mode = 0b0001
    publ.Write(cmd)
    pubr.Write(cmd)
    print("  Sent: [0, 0, 0, 0, 0, 1000]")
    time.sleep(3.0)
    
    print("\n=== Starting test loop (like working example) ===")
    short_value = 1000
    
    for cnd in range(50):  # Just 50 cycles for testing
        if (cnd + 1) % 10 == 0:
            short_value = 1000 - short_value
            
        values_to_write = [short_value] * 6
        values_to_write[-1] = 1000 - values_to_write[-1]
        values_to_write[-2] = 1000 - values_to_write[-2]
        
        value_to_write_np = np.array(values_to_write)
        value_to_write_np = np.clip(value_to_write_np, 200, 800)
        
        cmd.angle_set = value_to_write_np.tolist()
        cmd.mode = 0b0001
        
        if publ.Write(cmd) and pubr.Write(cmd):
            if cnd % 10 == 0:
                print(f"  [{cnd:3d}] Sent: {cmd.angle_set}")
        else:
            print(f"  [{cnd:3d}] Write FAILED - waiting for subscriber")
        
        time.sleep(0.1)
    
    print("\n✓ Test complete. Did hands move?")
