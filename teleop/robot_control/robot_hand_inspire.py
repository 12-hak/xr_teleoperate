from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize # dds
from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorCmds_, MotorStates_                           # idl
from unitree_sdk2py.idl.default import unitree_go_msg_dds__MotorCmd_
from teleop.robot_control.hand_retargeting import HandRetargeting, HandType
import numpy as np
from enum import IntEnum
import threading
import time
from multiprocessing import Process, Array, Value
from inspire_sdkpy import inspire_dds  # lazy import
import inspire_sdkpy.inspire_hand_defaut as inspire_hand_default

import logging_mp
logger_mp = logging_mp.get_logger(__name__)

Inspire_Num_Motors = 6
kTopicInspireDFXCommand = "rt/inspire/cmd"
kTopicInspireDFXState = "rt/inspire/state"

class Inspire_Controller_DFX:
    def __init__(self, left_hand_array, right_hand_array, dual_hand_data_lock = None, dual_hand_state_array = None,
                       dual_hand_action_array = None, fps = 100.0, Unit_Test = False, simulation_mode = False, network_interface = None):
        logger_mp.info("Initialize Inspire_Controller_DFX...")
        self.fps = fps
        self.Unit_Test = Unit_Test
        self.simulation_mode = simulation_mode
        self.network_interface = network_interface
        if not self.Unit_Test:
            self.hand_retargeting = HandRetargeting(HandType.INSPIRE_HAND)
        else:
            self.hand_retargeting = HandRetargeting(HandType.INSPIRE_HAND_Unit_Test)


        # initialize handcmd publisher and handstate subscriber
        self.HandCmb_publisher = ChannelPublisher(kTopicInspireDFXCommand, MotorCmds_)
        self.HandCmb_publisher.Init()

        self.HandState_subscriber = ChannelSubscriber(kTopicInspireDFXState, MotorStates_)
        self.HandState_subscriber.Init()

        # Shared Arrays for hand states
        self.left_hand_state_array  = Array('d', Inspire_Num_Motors, lock=True)  
        self.right_hand_state_array = Array('d', Inspire_Num_Motors, lock=True)

        # initialize subscribe thread
        self.subscribe_state_thread = threading.Thread(target=self._subscribe_hand_state)
        self.subscribe_state_thread.daemon = True
        self.subscribe_state_thread.start()

        while True:
            if any(self.right_hand_state_array): # any(self.left_hand_state_array) and 
                break
            time.sleep(0.01)
            logger_mp.warning("[Inspire_Controller_DFX] Waiting to subscribe dds...")
        logger_mp.info("[Inspire_Controller_DFX] Subscribe dds ok.")

        hand_control_thread = threading.Thread(target=self.control_process, args=(left_hand_array, right_hand_array,  self.left_hand_state_array, self.right_hand_state_array,
                                                                          dual_hand_data_lock, dual_hand_state_array, dual_hand_action_array))
        hand_control_thread.daemon = True
        hand_control_thread.start()

        logger_mp.info("Initialize Inspire_Controller_DFX OK!")

    def _subscribe_hand_state(self):
        while True:
            hand_msg  = self.HandState_subscriber.Read()
            if hand_msg is not None:
                for idx, id in enumerate(Inspire_Left_Hand_JointIndex):
                    self.left_hand_state_array[idx] = hand_msg.states[id].q
                for idx, id in enumerate(Inspire_Right_Hand_JointIndex):
                    self.right_hand_state_array[idx] = hand_msg.states[id].q
            time.sleep(0.002)

    def ctrl_dual_hand(self, left_q_target, right_q_target):
        """
        Set current left, right hand motor state target q
        """
        for idx, id in enumerate(Inspire_Left_Hand_JointIndex):             
            self.hand_msg.cmds[id].q = left_q_target[idx]         
        for idx, id in enumerate(Inspire_Right_Hand_JointIndex):             
            self.hand_msg.cmds[id].q = right_q_target[idx] 

        self.HandCmb_publisher.Write(self.hand_msg)
        # logger_mp.debug("hand ctrl publish ok.")
    
    def control_process(self, left_hand_array, right_hand_array, left_hand_state_array, right_hand_state_array,
                              dual_hand_data_lock = None, dual_hand_state_array = None, dual_hand_action_array = None):
        self.running = True

        left_q_target  = np.full(Inspire_Num_Motors, 1.0)
        right_q_target = np.full(Inspire_Num_Motors, 1.0)
        
        debug_counter = 0  # For periodic debug output

        # initialize inspire hand's cmd msg
        self.hand_msg  = MotorCmds_()
        self.hand_msg.cmds = [unitree_go_msg_dds__MotorCmd_() for _ in range(len(Inspire_Right_Hand_JointIndex) + len(Inspire_Left_Hand_JointIndex))]

        for idx, id in enumerate(Inspire_Left_Hand_JointIndex):
            self.hand_msg.cmds[id].q = 1.0
        for idx, id in enumerate(Inspire_Right_Hand_JointIndex):
            self.hand_msg.cmds[id].q = 1.0

        try:
            while self.running:
                start_time = time.time()
                # get dual hand state
                with left_hand_array.get_lock():
                    left_hand_data  = np.array(left_hand_array[:]).reshape(25, 3).copy()
                with right_hand_array.get_lock():
                    right_hand_data = np.array(right_hand_array[:]).reshape(25, 3).copy()

                # Read left and right q_state from shared arrays
                state_data = np.concatenate((np.array(left_hand_state_array[:]), np.array(right_hand_state_array[:])))

                # Debug: Print every 30 frames (~1 second at 100Hz)
                debug_counter += 1
                show_debug = (debug_counter % 100 == 0)
                
                hand_data_valid = not np.all(right_hand_data == 0.0) and not np.all(left_hand_data[4] == np.array([-1.13, 0.3, 0.15]))
                
                if show_debug:
                    logger_mp.info(f"[DEBUG DFX] Hand data valid: {hand_data_valid}")
                    if not hand_data_valid:
                        logger_mp.info(f"[DEBUG DFX]   Right hand all zeros: {np.all(right_hand_data == 0.0)}")
                        logger_mp.info(f"[DEBUG DFX]   Left hand[4]: {left_hand_data[4]}")

                if hand_data_valid: # if hand data has been initialized.
                    ref_left_value = left_hand_data[self.hand_retargeting.left_indices[1,:]] - left_hand_data[self.hand_retargeting.left_indices[0,:]]
                    ref_right_value = right_hand_data[self.hand_retargeting.right_indices[1,:]] - right_hand_data[self.hand_retargeting.right_indices[0,:]]

                    if show_debug:
                        logger_mp.info(f"[DEBUG DFX] Ref values - Left shape: {ref_left_value.shape}, Right shape: {ref_right_value.shape}")
                        logger_mp.info(f"[DEBUG DFX] Ref left sample: {ref_left_value[0]}")

                    left_q_target  = self.hand_retargeting.left_retargeting.retarget(ref_left_value)[self.hand_retargeting.left_dex_retargeting_to_hardware]
                    right_q_target = self.hand_retargeting.right_retargeting.retarget(ref_right_value)[self.hand_retargeting.right_dex_retargeting_to_hardware]
                    
                    if show_debug:
                        logger_mp.info(f"[DEBUG DFX] Retargeting output - Left: {left_q_target}")
                        logger_mp.info(f"[DEBUG DFX] Retargeting output - Right: {right_q_target}")

                    # In website https://support.unitree.com/home/en/G1_developer/inspire_dfx_dexterous_hand, you can find
                    #     In the official document, the angles are in the range [0, 1] ==> 0.0: fully closed  1.0: fully open
                    # The q_target now is in radians, ranges:
                    #     - idx 0~3: 0~1.7 (1.7 = closed)
                    #     - idx 4:   0~0.5
                    #     - idx 5:  -0.1~1.3
                    # We normalize them using (max - value) / range
                    def normalize(val, min_val, max_val):
                        return np.clip((max_val - val) / (max_val - min_val), 0.0, 1.0)

                    for idx in range(Inspire_Num_Motors):
                        if idx <= 3:
                            left_q_target[idx]  = normalize(left_q_target[idx], 0.0, 1.7)
                            right_q_target[idx] = normalize(right_q_target[idx], 0.0, 1.7)
                        elif idx == 4:
                            left_q_target[idx]  = normalize(left_q_target[idx], 0.0, 0.5)
                            right_q_target[idx] = normalize(right_q_target[idx], 0.0, 0.5)
                        elif idx == 5:
                            left_q_target[idx]  = normalize(left_q_target[idx], -0.1, 1.3)
                            right_q_target[idx] = normalize(right_q_target[idx], -0.1, 1.3)
                    
                    if show_debug:
                        logger_mp.info(f"[DEBUG DFX] Normalized - Left: {left_q_target}, Right: {right_q_target}")

                # get dual hand action
                action_data = np.concatenate((left_q_target, right_q_target))    
                if dual_hand_state_array and dual_hand_action_array:
                    with dual_hand_data_lock:
                        dual_hand_state_array[:] = state_data
                        dual_hand_action_array[:] = action_data

                self.ctrl_dual_hand(left_q_target, right_q_target)
                current_time = time.time()
                time_elapsed = current_time - start_time
                sleep_time = max(0, (1 / self.fps) - time_elapsed)
                time.sleep(sleep_time)
        finally:
            logger_mp.info("Inspire_Controller_DFX has been closed.")

# for inspire-ftp
Inspire_Num_Motors = 6
kTopicInspireFTPLeftCommand = "rt/inspire_hand/ctrl/l"
kTopicInspireFTPRightCommand = "rt/inspire_hand/ctrl/r"
kTopicInspireFTPLeftState = "rt/inspire_hand/state/l"
kTopicInspireFTPRightState = "rt/inspire_hand/state/r"

class Inspire_Controller_FTP:
    def __init__(self, left_hand_array, right_hand_array, dual_hand_data_lock = None, dual_hand_state_array = None,
                       dual_hand_action_array = None, fps = 100.0, Unit_Test = False, simulation_mode = False, network_interface = None):
        logger_mp.info("Initialize Inspire_Controller_FTP...")
        # from inspire_sdkpy import inspire_dds  # lazy import
        # import inspire_sdkpy.inspire_hand_defaut as inspire_hand_default
        self.fps = fps
        self.Unit_Test = Unit_Test
        self.simulation_mode = simulation_mode
        self.network_interface = network_interface
        if not self.Unit_Test:
            self.hand_retargeting = HandRetargeting(HandType.INSPIRE_HAND)
        else:
            self.hand_retargeting = HandRetargeting(HandType.INSPIRE_HAND_Unit_Test)


        # DON'T create publishers here - they must be created in the subprocess!
        # Shared Arrays for hand states ([0,1] normalized values)
        self.left_hand_state_array  = Array('d', Inspire_Num_Motors, lock=True)
        self.right_hand_state_array = Array('d', Inspire_Num_Motors, lock=True)
        self._state_received = Value('b', False) # Boolean flag

        # Initialize subscribe thread (runs in main process)
        self.subscribe_state_thread = threading.Thread(target=self._subscribe_hand_state)
        self.subscribe_state_thread.daemon = True
        self.subscribe_state_thread.start()

        # Wait for initial DDS messages
        wait_count = 0
        while not self._state_received.value:
            if wait_count % 100 == 0: # Print every second
                logger_mp.info(f"[Inspire_Controller_FTP] Waiting for first DDS state message...")
            time.sleep(0.01)
            wait_count += 1
            if wait_count > 300: # 3 second timeout
                logger_mp.warning("[Inspire_Controller_FTP] Timeout waiting for hand state. Proceeding...")
                break

        hand_control_thread = threading.Thread(target=self.control_process, args=(left_hand_array, right_hand_array, self.left_hand_state_array, self.right_hand_state_array,
                                                                          dual_hand_data_lock, dual_hand_state_array, dual_hand_action_array))
        hand_control_thread.daemon = True
        hand_control_thread.start()

        logger_mp.info("Initialize Inspire_Controller_FTP OK!\n")

    def _subscribe_hand_state(self):
        logger_mp.info("[Inspire_Controller_FTP] Subscribe thread started.")
        
        # Create subscribers in this thread (main process)
        left_sub = ChannelSubscriber(kTopicInspireFTPLeftState, inspire_dds.inspire_hand_state)
        left_sub.Init()
        right_sub = ChannelSubscriber(kTopicInspireFTPRightState, inspire_dds.inspire_hand_state)
        right_sub.Init()
        
        state_recv_count = 0
        while True:
            # Left Hand
            left_state_msg = left_sub.Read()
            if left_state_msg is not None:
                if hasattr(left_state_msg, 'angle_act') and len(left_state_msg.angle_act) == Inspire_Num_Motors:
                    with self.left_hand_state_array.get_lock():
                        for i in range(Inspire_Num_Motors):
                            self.left_hand_state_array[i] = left_state_msg.angle_act[i] / 1000.0
                    state_recv_count += 1
                else:
                    logger_mp.warning(f"[Inspire_Controller_FTP] Received left_state_msg but attributes are missing or incorrect. Type: {type(left_state_msg)}, Content: {str(left_state_msg)[:100]}")
            # Right Hand
            right_state_msg = right_sub.Read()
            if right_state_msg is not None:
                if hasattr(right_state_msg, 'angle_act') and len(right_state_msg.angle_act) == Inspire_Num_Motors:
                    with self.right_hand_state_array.get_lock():
                        for i in range(Inspire_Num_Motors):
                            self.right_hand_state_array[i] = right_state_msg.angle_act[i] / 1000.0
                else:
                    logger_mp.warning(f"[Inspire_Controller_FTP] Received right_state_msg but attributes are missing or incorrect. Type: {type(right_state_msg)}, Content: {str(right_state_msg)[:100]}")
            
            if left_state_msg is not None or right_state_msg is not None:
                self._state_received.value = True

            # Debug: Log state reception every 500 messages
            if state_recv_count % 500 == 0 and state_recv_count > 0:
                logger_mp.info(f"[STATE FTP] Received {state_recv_count} state messages from hands")
            
            time.sleep(0.002)

    def control_process(self, left_hand_array, right_hand_array, left_hand_state_array, right_hand_state_array,
                              dual_hand_data_lock = None, dual_hand_state_array = None, dual_hand_action_array = None):

        logger_mp.info("[Inspire_Controller_FTP] Control process started.")
        self.running = True

        # CRITICAL: Create publishers in THIS process (not main process)!
        logger_mp.info("[Inspire_Controller_FTP] Creating DDS publishers in subprocess...")
        left_pub = ChannelPublisher(kTopicInspireFTPLeftCommand, inspire_dds.inspire_hand_ctrl)
        left_pub.Init()
        right_pub = ChannelPublisher(kTopicInspireFTPRightCommand, inspire_dds.inspire_hand_ctrl)
        right_pub.Init()
        time.sleep(0.1) # Give some time for discovery
        
        # Send initialization sequence
        logger_mp.info("[Inspire_Controller_FTP] Sending initialization sequence...")
        init_cmd = inspire_hand_default.get_inspire_hand_ctrl()
        
        # Set motor speed to maximum (1000)
        init_cmd.speed_set = [1000] * 6
        init_cmd.mode = 0b1000 # Mode 8: Speed limit
        left_pub.Write(init_cmd)
        right_pub.Write(init_cmd)
        time.sleep(0.1)
        
        # First initialization command (Angles)
        init_cmd.angle_set = [0, 0, 0, 0, 1000, 1000]
        init_cmd.mode = 0b0001
        left_pub.Write(init_cmd)
        right_pub.Write(init_cmd)
        time.sleep(1.0)
        
        # Second initialization command  
        init_cmd.angle_set = [0, 0, 0, 0, 0, 1000]
        init_cmd.mode = 0b0001
        left_pub.Write(init_cmd)
        right_pub.Write(init_cmd)
        time.sleep(3.0)
        
        logger_mp.info("[Inspire_Controller_FTP] Initialization complete. Starting control loop...")

        left_q_target  = np.full(Inspire_Num_Motors, 1.0)
        right_q_target = np.full(Inspire_Num_Motors, 1.0)
        
        # Prepare command objects for reuse
        left_cmd = inspire_hand_default.get_inspire_hand_ctrl()
        right_cmd = inspire_hand_default.get_inspire_hand_ctrl()
        left_cmd.mode = 0b0001
        right_cmd.mode = 0b0001
        
        debug_counter = 0  # For periodic debug output
        publish_count = 0  # For debug logging

        try:
            while self.running:
                start_time = time.time()
                # get dual hand state
                with left_hand_array.get_lock():
                    left_hand_data  = np.array(left_hand_array[:]).reshape(25, 3).copy()
                with right_hand_array.get_lock():
                    right_hand_data = np.array(right_hand_array[:]).reshape(25, 3).copy()

                # Read left and right q_state from shared arrays
                state_data = np.concatenate((np.array(left_hand_state_array[:]), np.array(right_hand_state_array[:])))

                # Debug: Print every 30 frames (~0.3 seconds at 100Hz)
                debug_counter += 1
                show_debug = (debug_counter % 30 == 0)
                
                hand_data_valid = not np.all(right_hand_data == 0.0) and not np.all(left_hand_data[4] == np.array([-1.13, 0.3, 0.15]))
                
                if show_debug and debug_counter % 300 == 0:
                    logger_mp.info(f"[DEBUG FTP] Hand data valid: {hand_data_valid}")
                    if not hand_data_valid:
                        logger_mp.info(f"[DEBUG FTP]   Right hand all zeros: {np.all(right_hand_data == 0.0)}")
                        logger_mp.info(f"[DEBUG FTP]   Left hand[4]: {left_hand_data[4]}")

                if hand_data_valid: # if hand data has been initialized.
                    ref_left_value = left_hand_data[self.hand_retargeting.left_indices[1,:]] - left_hand_data[self.hand_retargeting.left_indices[0,:]]
                    ref_right_value = right_hand_data[self.hand_retargeting.right_indices[1,:]] - right_hand_data[self.hand_retargeting.right_indices[0,:]]

                    if show_debug and debug_counter % 300 == 0:
                        logger_mp.info(f"[DEBUG FTP] Ref values - Left shape: {ref_left_value.shape}, Right shape: {ref_right_value.shape}")
                        logger_mp.info(f"[DEBUG FTP] Ref left sample: {ref_left_value[0]}")

                    left_q_target  = self.hand_retargeting.left_retargeting.retarget(ref_left_value)[self.hand_retargeting.left_dex_retargeting_to_hardware]
                    right_q_target = self.hand_retargeting.right_retargeting.retarget(ref_right_value)[self.hand_retargeting.right_dex_retargeting_to_hardware]
                    
                    if show_debug and debug_counter % 300 == 0:
                        logger_mp.info(f"[DEBUG FTP] Retargeting output - Left: {left_q_target}")
                        logger_mp.info(f"[DEBUG FTP] Retargeting output - Right: {right_q_target}")

                    def normalize(val, min_val, max_val):
                        return np.clip((max_val - val) / (max_val - min_val), 0.0, 1.0)

                    for idx in range(Inspire_Num_Motors):
                        if idx <= 3:
                            left_q_target[idx]  = normalize(left_q_target[idx], 0.0, 1.7)
                            right_q_target[idx] = normalize(right_q_target[idx], 0.0, 1.7)
                        elif idx == 4:
                            left_q_target[idx]  = normalize(left_q_target[idx], 0.0, 0.5)
                            right_q_target[idx] = normalize(right_q_target[idx], 0.0, 0.5)
                        elif idx == 5:
                            left_q_target[idx]  = normalize(left_q_target[idx], -0.1, 1.3)
                            right_q_target[idx] = normalize(right_q_target[idx], -0.1, 1.3)
                
                # CRITICAL: Expanded range to 0-1000 per user request
                scaled_left_cmd = [int(np.clip(val * 1000, 0, 1000)) for val in left_q_target]
                scaled_right_cmd = [int(np.clip(val * 1000, 0, 1000)) for val in right_q_target]

                if show_debug and debug_counter % 300 == 0:
                    logger_mp.info(f"[DEBUG FTP] Normalized - Left: {left_q_target}")
                    logger_mp.info(f"[DEBUG FTP] Scaled commands - Left: {scaled_left_cmd}, Right: {scaled_right_cmd}")
                    logger_mp.info(f"[DEBUG FTP] Current hand state - Left: {list(left_hand_state_array[:])}, Right: {list(right_hand_state_array[:])}")
                    if np.all(np.array(left_hand_state_array[:]) == 0.0):
                        logger_mp.warning("[DEBUG FTP] Subscriber state is ALL ZEROS. Check if driver is sending state!")

                # get dual hand action
                action_data = np.concatenate((left_q_target, right_q_target))
                if dual_hand_state_array and dual_hand_action_array:
                    with dual_hand_data_lock:
                        dual_hand_state_array[:] = state_data
                        dual_hand_action_array[:] = action_data

                # Send commands directly
                left_cmd.angle_set = scaled_left_cmd
                right_cmd.angle_set = scaled_right_cmd
                
                # Write and check success
                left_ok = left_pub.Write(left_cmd)
                right_ok = right_pub.Write(right_cmd)
                
                # Debug logging reduced to match 30Hz
                publish_count += 1
                if publish_count % 300 == 0:
                    status = "✓" if (left_ok and right_ok) else "✗"
                    logger_mp.info(f"[PUBLISH FTP {status}] Published {publish_count} commands")
                    if not left_ok:
                        logger_mp.warning("[PUBLISH FTP] Left hand Write() returned False - no subscriber?")
                    if not right_ok:
                        logger_mp.warning("[PUBLISH FTP] Right hand Write() returned False - no subscriber?")
                current_time = time.time()
                time_elapsed = current_time - start_time
                sleep_time = max(0, (1 / self.fps) - time_elapsed)
                time.sleep(sleep_time)
        finally:
            logger_mp.info("Inspire_Controller_FTP has been closed.")

# Update hand state, according to the official documentation:
# 1. https://support.unitree.com/home/en/G1_developer/inspire_dfx_dexterous_hand
# 2. https://support.unitree.com/home/en/G1_developer/inspire_ftp_dexterity_hand
# the state sequence is as shown in the table below
# ┌──────┬───────┬──────┬────────┬────────┬────────────┬────────────────┬───────┬──────┬────────┬────────┬────────────┬────────────────┐
# │ Id   │   0   │  1   │   2    │   3    │     4      │       5        │   6   │  7   │   8    │   9    │    10      │       11       │
# ├──────┼───────┼──────┼────────┼────────┼────────────┼────────────────┼───────┼──────┼────────┼────────┼────────────┼────────────────┤
# │      │                    Right Hand                                │                   Left Hand                                  │
# │Joint │ pinky │ ring │ middle │ index  │ thumb-bend │ thumb-rotation │ pinky │ ring │ middle │ index  │ thumb-bend │ thumb-rotation │
# └──────┴───────┴──────┴────────┴────────┴────────────┴────────────────┴───────┴──────┴────────┴────────┴────────────┴────────────────┘
class Inspire_Right_Hand_JointIndex(IntEnum):
    kRightHandPinky = 0
    kRightHandRing = 1
    kRightHandMiddle = 2
    kRightHandIndex = 3
    kRightHandThumbBend = 4
    kRightHandThumbRotation = 5

class Inspire_Left_Hand_JointIndex(IntEnum):
    kLeftHandPinky = 0
    kLeftHandRing = 1
    kLeftHandMiddle = 2
    kLeftHandIndex = 3
    kLeftHandThumbBend = 4
    kLeftHandThumbRotation = 5
