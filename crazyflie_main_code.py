#turning direction and speed, should be changed depending on the layout of arena

import logging
import time
import random

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper
from cflib.utils.reset_estimator import reset_estimator
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.high_level_commander import HighLevelCommander 

#Address Initialization
URI = uri_helper.uri_from_env(default='radio://0/98/2M/E7E7E7E7E8')
logging.basicConfig(level=logging.ERROR)

# --- GLOBAL STATE (shared across functions) ---
TARGET_HEIGHT_M = 0.4572  # Target height in meters (~1.5 feet)s
CONFIRMATION_DURATION = 0.25  # seconds to confirm landing pad detection
pad_detection_start_time = None # Landing pad initilization
initial_x = 0 # Initial position for boundary x checks
initial_y = 0 # Initial position for boundary y checks
center_x = 0 
center_y = 0
check_point = 0
i = 0
j = 0
Run = 1
y_bound = 1
# Shared state container for log data
log_data = {"front": None, "left": None, "right": None, "down": None, "x": None, "y": None}

# --------------------------------------------------- LOGGING CALLBACK ---------------------------------#
def log_callback(timestamp, data, logconf):
    """Callback to update the shared log_data dictionary."""
    global log_data
    
    # Range sensors (convert mm to m)
    for sensor in ['front', 'left', 'right', 'back', 'up']:
        value = data.get(f'range.{sensor}', 65535)
        if value < 65535:
            log_data[sensor] = value / 1000.0
        else:
            log_data[sensor] = None

    # Downward range (zrange)
    down_value = data.get('range.zrange', 65535)
    if down_value < 65535:
        log_data["down"] = down_value / 1000.0
    else:
        log_data["down"] = None

    # State Estimate (position)
    log_data["x"] = data.get('stateEstimate.x', None)
    log_data["y"] = data.get('stateEstimate.y', None)


#----------------------------------------------OBSTACLE AVOIDANCE FUNCTION----------------------------------------------------
def Obstacle_Avoid(cf, TARGET_HEIGHT_M):
    print("Starting obstacle avoidance...")

    #All gloabl variables carried over for this function
    global i,j, initial_x, initial_y, center_x, center_y, check_point, Run, og_x, og_y, y_bound
    global pad_detection_start_time, CONFIRMATION_DURATION 

    # Reset detection state at the start of the function
    pad_detection_start_time = None 
    
    # Wait for initial position and sensor data
    print("Waiting for initial data...")
    while log_data["x"] is None or log_data["front"] is None:
        time.sleep(0.1)
    
    if Run == 1:
        y_yaw = random.choice([-70, -70])
        y_bound = 1
    elif Run == 2:
        y_yaw = random.choice([80, 80])
        y_bound = 1

    #Record original position for return boundary condition only once
    if j == 0:
        og_x = log_data["x"]
        og_y = log_data["y"]
        j = 1

    #Log initial position for boundary checks for each run
    if i == 0:
        initial_x = log_data["x"]
        print(f"{initial_x}, {initial_y}")
        initial_y = log_data["y"]
        i = 1
   
   #Constant loop for obstacle avoidance and landing pad detection  
    while True:

        front_distance = log_data.get("front")
        left_distance = log_data.get("left")
        right_distance = log_data.get("right")
        down_distance = log_data.get("down")
        current_x = log_data.get("x")
        current_y = log_data.get("y")

        print(f"Front: {front_distance}, Down: {down_distance} | Pos: x={current_x:.2f}, y={current_y:.2f}")

        front_distance = log_data.get("front")
        down_distance = log_data.get("down")
        current_x = log_data.get("x")
        current_y = log_data.get("y")

        print(f"Front: {front_distance}, Down: {down_distance} | Pos: x={current_x:.2f}, y={current_y:.2f}")

        #Setting time variable for monitoring landing pad detection
        current_time = time.time()

        # --- LANDING PAD DETECTION LOGIC ---
        #If down sensor sees altitide change above thereshold, start/continue confirmation timer
        if down_distance is not None and down_distance > 0.51:
                
                # if First time the condition is met: start the timer
                if pad_detection_start_time is None:                    
                    pad_detection_start_time = current_time
                    print("Initial landing pad detection. Starting timer...")
                
                # Check if the condition has been met for the required duration (CONFIRMATION_DURATION)
                elif current_time - pad_detection_start_time >= CONFIRMATION_DURATION:           
                    print(f"Landing pad confirmed after {CONFIRMATION_DURATION}s! Stopping horizontal movement.")
                    Run += 1
                    
                    # Stop momentum for landing function
                    cf.commander.send_hover_setpoint(0.2, 0, 0, TARGET_HEIGHT_M)
                    time.sleep(0.1)
                    for _ in range(5):
                        cf.commander.send_hover_setpoint(-0.15, 0, 0, TARGET_HEIGHT_M)
                        time.sleep(0.1)
                    for _ in range(1):
                        cf.commander.send_hover_setpoint(0, 0, 0, TARGET_HEIGHT_M)
                        time.sleep(0.1)
                    
                    # Exit the Obstacle_Avoid loop
                    break

            
        else:
                # If the condition is not met, reset the timer
                if pad_detection_start_time is not None:
                    print("Landing pad condition lost. Resetting timer.")
                    pad_detection_start_time = None
                    
        # Default: move forward
        roll, pitch, yaw = 0, 0.2, 0 

        # If front snesor detects, stop forward motion and prepare to turn
        if front_distance is not None and front_distance < 0.3:

            # Retrieve side distances for decision making
            left = log_data.get("left")
            right = log_data.get("right")
        
            #Turn toward the side with the most clearance
            if left is not None and right is not None:
                if left > right + 0.1:
                    roll, pitch, yaw = 0, 0, 60 # Turn Left (Negative Yaw)
                    print("Obstacle Front, turning LEFT (clearance on left)")
                elif right > left + 0.1:
                    roll, pitch, yaw = 0, 0, -60   # Turn Right (Positive Yaw)
                    print("Obstacle Front, turning RIGHT (clearance on right)")
                else:
                    # if Sides are equally close/far or both blocked. Random turn.
                    yaw = random.choice([60, -60])
                    roll, pitch = 0, 0
                    print(f"Obstacle Front, equal clearance, turning {('RIGHT' if yaw > 0 else 'LEFT')}")

        # If side sensors detect, strafe away from obstacle
        elif left_distance is not None and left_distance < 0.35:
            roll, pitch, yaw = -0.1, 0.2, 0  # STOP forward, TURN RIGHT
            print("Obstacle on left, move right")

        # If side sensors detect, strafe away from obstacle
        elif right_distance is not None and right_distance < 0.35:
            roll, pitch, yaw = 0.1, 0.2, 0 # STOP forward, TURN LEFT
            print("Obstacle on right, move left")
        
        # Y Boundary Checks. If met do 180 or half turn
        elif current_y is not None and current_y > y_bound:
            print("Boundary y reached, turning around")
            cf.commander.send_hover_setpoint(0,0,0,TARGET_HEIGHT_M)
            time.sleep(0.1)
            for _ in range (5):
                cf.commander.send_hover_setpoint(-0.5, 0, 0, TARGET_HEIGHT_M)
                time.sleep(0.1)
            cf.commander.send_hover_setpoint(0,0,0,TARGET_HEIGHT_M)
            time.sleep(0.1)
            roll, pitch, yaw = 0, 0, y_yaw
        
        # X Boundary Checks (RUN1 ONLY). If met do 180 or half turn
        elif current_x is not None and current_x < initial_x-0.2 and Run == 1:
            print(f"Boundary {check_point} back reached, turning around")
            cf.commander.send_hover_setpoint(0,0,0,TARGET_HEIGHT_M)
            time.sleep(0.1)
            for _ in range (10):
                cf.commander.send_hover_setpoint(-0.5, 0, 0, TARGET_HEIGHT_M)
                time.sleep(0.1)
            cf.commander.send_hover_setpoint(0,0,0,TARGET_HEIGHT_M)
            time.sleep(0.1)
            roll, pitch, yaw = 0, 0, random.choice([105, 80])  

        # X Boundary Checks (RUN2 ONLY). If met do 180 or half turn
        elif current_x is not None and current_x > initial_x+0.2 and Run == 2:
            print(f"Boundary {check_point} back reached, turning around")
            cf.commander.send_hover_setpoint(0,0,0,TARGET_HEIGHT_M)
            time.sleep(0.1)
            for _ in range (10):
                cf.commander.send_hover_setpoint(-0.5, 0, 0, TARGET_HEIGHT_M)
                time.sleep(0.1)
            cf.commander.send_hover_setpoint(0,0,0,TARGET_HEIGHT_M)
            time.sleep(0.1)
            roll, pitch, yaw = 0, 0, random.choice([105, 80])

        # X Boundary Checks for RUN2 pad boundary. If met do 180 or half turn
        elif current_x is not None and current_x < og_x - 1 and Run == 2:
            print(f"Boundary far back reached, turning around")
            cf.commander.send_hover_setpoint(0,0,0,TARGET_HEIGHT_M)
            time.sleep(0.1)
            for _ in range (10):
                cf.commander.send_hover_setpoint(-0.5, 0, 0, TARGET_HEIGHT_M)
                time.sleep(0.1)
            cf.commander.send_hover_setpoint(0,0,0,TARGET_HEIGHT_M)
            time.sleep(0.1)
            roll, pitch, yaw = 0, 0, random.choice([105, 80])   

# Run 1 Boundary Updates
        
        if Run == 1:
            # Update 1.25m Boundary (RUN 1)
            if current_x is not None and initial_x != 1.25 and check_point == 0 and current_x > 1.25:
                initial_x = 1.25 
                check_point = 1
                print("Boundary Updated to 1.25m (checkpoint 1)")
                print(f"{initial_x}, {initial_y}")
            
            # Update 2m Boundary (RUN 1)
            if current_x is not None and initial_x == 1.25 and initial_x != 2 and check_point == 1 and current_x > 2:
                initial_x = 2
                check_point = 2
                print("Boundary Updated to 2m (checkpoint 2)")
                print(f"{initial_x}, {initial_y}")

            # Update 3m Boundary (RUN 1)
            if current_x is not None and initial_x == 2 and initial_x != 3 and check_point == 2 and current_x > 3:
                initial_x = 3
                check_point = 3
                print("Boundary Updated to 3m (checkpoint 3)")
                print(f"{initial_x}, {initial_y}")

             # Update 4m Boundary (RUN 1)
            if current_x is not None and initial_x == 3 and initial_x != 4 and check_point == 3 and current_x > 4:
                initial_x = 4
                check_point = 4
                print("Boundary Updated to 4m (checkpoint 4)")
                print(f"{initial_x}, {initial_y}")
            
            # Y boundary update
            if current_x is not None and check_point == 4 and current_x > 4 and current_y < 0.5:
                y_bound = 0.5
                check_point = 5
                print("Boundary Updated to y bound")
                print(f"{initial_x}, {initial_y}")


# Run 2 Boundary Updates   
        if Run == 2:
            # Update 3m Boundary (RUN 2)
            if current_x is not None and check_point == 0 and current_x < 3: 
                initial_x = 3
                check_point = 1
                print("Boundary Updated to 3m (Checkpoint 1)") 

            # # Update 2m Boundary (RUN 2)
            if  current_x is not None and initial_x == 3 and check_point == 1 and current_x < 2:
                initial_x = 2
                check_point = 2
                print("Boundary Updated to 2m (Checkpoint 2)")

            # # Update 4m Boundary (RUN 2)
            if  current_x is not None and initial_x == 2 and check_point == 2 and current_x < 1:
                initial_x = 1
                check_point = 3
                print("Boundary Updated to 1m (Checkpoint 3)")

            # # Update 0.4m Boundary (RUN 2)
            if  current_x is not None and initial_x == 1 and check_point == 3 and current_x < 0.4:
                initial_x = 0.4
                check_point = 4
                print("Boundary Updated to 0.4m (Checkpoint 4)")
            
            # Y boundary update
            if current_x is not None and check_point == 4 and current_x > 4 and current_y < 0.5:
                y_bound = 0.5
                y_yaw = random.choice([-105, -70])
                check_point = 5
                print("Boundary Updated to y bound")
                print(f"{initial_x}, {initial_y}")

            

        
#----------------------------------------------End Boundary Updates----------------------------------------------------

            
        # Send the selected setpoint (default or avoidance) for 1.5 seconds
        if yaw != 0:
            for _ in range(15): 
                cf.commander.send_hover_setpoint(pitch, roll, yaw, TARGET_HEIGHT_M)
                time.sleep(0.1)
        else:
            # If no obstacle/turning, just move forward/hold
            cf.commander.send_hover_setpoint(pitch, roll, yaw, TARGET_HEIGHT_M)
            time.sleep(0.1)

        #Update i to always ensure initial x/y are only set once per run
        i += 1
    return center_x, center_y


# -------------------------------------------------- LANDING AND RELAUNCH FUNCTION ----------------------------------------------
def Land_or_Relaunch(cf, TARGET_HEIGHT_M):
    print("Landing now...")

    global Run
    COMMAND_RATE = 100 # Hz of how many setpoints are sent per second
    TARGET_HEIGHT = 0.5   
    TAKEOFF_DURATION = 2.0 # seconds to ramp up to target height
    LANDING_DURATION = 1.5 # seconds to land  
    current_height = TARGET_HEIGHT      
    dt = 1.0 / COMMAND_RATE #actaual time step based on command rate
    takeoff_steps = int(TAKEOFF_DURATION * COMMAND_RATE)
    landing_steps = int(LANDING_DURATION * COMMAND_RATE)
    takeoff_height_delta = TARGET_HEIGHT / takeoff_steps
    landing_height_delta = TARGET_HEIGHT / landing_steps

    for _ in range(landing_steps):
        current_height -= landing_height_delta
        clamped_height = max(0.0, current_height) # Height must not go below 0
        cf.commander.send_hover_setpoint(0, 0, 0, clamped_height)
        time.sleep(dt)
        
    # Ensure motors are off and wait on the ground
    cf.commander.send_stop_setpoint()
    time.sleep(1.0) 

    # Relaunch Sequence if on run 2
    if Run == 2:
        # --- PHASE 3: SECOND TAKE OFF & HOVER ---
        print(f"3/4: Taking off again to {TARGET_HEIGHT}m...")
        current_height = 0.0
    
        try:    
            # Ramp up height
            for _ in range(takeoff_steps):
                current_height += takeoff_height_delta
                clamped_height = min(current_height, TARGET_HEIGHT)
                cf.commander.send_hover_setpoint(0, 0, 0, clamped_height)
                time.sleep(dt)

            # Use the high level commander to land for a smooth touchdown
        except Exception as e:
            print(f"Takeoff failed: {e}")
       
        # Onec taken off turn around
        for _ in range(15):
            cf.commander.send_hover_setpoint(0, 0, 115, clamped_height)
            time.sleep(0.1)
        for _ in range(20): # Hold position for 0.5 seconds to kill momentum
            cf.commander.send_hover_setpoint(0.2, 0, 0, clamped_height)
            time.sleep(0.1)
        for _ in range(5): # Hold position for 0.5 seconds to kill momentum
            cf.commander.send_hover_setpoint(-0.2, 0, 0, clamped_height)
            time.sleep(0.1)
        for _ in range(15): # Hold position for 0.5 seconds to kill momentum
            cf.commander.send_hover_setpoint(0, 0, 0, TARGET_HEIGHT_M)
            time.sleep(0.1)

# -------------------------------------------------- TAKEOFF FUNCTION ----------------------------------------------
def Takeoff(cf, TARGET_HEIGHT_M):
    print("Taking off...")
    # Use the high level commander for a safe and controlled takeoff
    cf.high_level_commander.takeoff(0.2, 0.5)
    time.sleep(3) # Wait for takeoff to stabilize

    for _ in range(2):
         cf.commander.send_hover_setpoint(0, 0, 0, 0.2)
         time.sleep(0.1)
    for _ in range(25): # Hold position for 0.5 seconds to kill momentum
        cf.commander.send_hover_setpoint(0.2, 0, 0, 0.2)
        time.sleep(0.1)
    for _ in range(5): # Hold position for 0.5 seconds to kill momentum
        cf.commander.send_hover_setpoint(-0.2, 0, 0, 0.2)
        time.sleep(0.1)
    for _ in range(15): # Hold position for 0.5 seconds to kill momentum
        cf.commander.send_hover_setpoint(0, 0, 0, TARGET_HEIGHT_M)
        time.sleep(0.1)

# -------------------------------------------------- SAFE RESET ESTIMATOR FUNCTION --------------------------------------------------
def safe_reset_estimator(scf, timeout=5):
    """
    Try to reset the Kalman estimator, but do not block forever.
    Will continue after `timeout` seconds even if variance is not low.
    """
    start_time = time.time()
    try:
        reset_estimator(scf)
        print("Estimator reset requested...")
    except Exception as e:
        print(f"Estimator reset failed: {e}")

    # Wait for convergence with timeout
    while time.time() - start_time < timeout:
        time.sleep(0.1)
    print("Estimator check timeout reached. Continuing anyway...")

# -------------------------------------------------- RESET GLOBALS FUNCTION -----------------------------------------------------------
def reset_globals():
    
    #Address all global variables that need to be reset for relaunch
    global initial_x, initial_y, center_x, center_y, check_point, i, pad_detection_start_time, log_data
    print("--- Resetting Global State for Relaunch ---")
        
    # Boundary/Checkpoint tracking
    check_point = 0
    
    # Initial position for boundary checks counter
    i = 0

    # Timer for confirming landing pad detection
    pad_detection_start_time = None
    
    # Dictionary to store log data (sensor and state estimates)
    log_data = {"front": None, "left": None, "right": None, "down": None, "x": None, "y": None}

# -------------------------------------------------- MAIN FUNCTION ----------------------------------------------------------------------

def main():
        # Initialize the low-level drivers
        cflib.crtp.init_drivers()
        with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
            cf = scf.cf

            # High Level Commander Setup
            cf.high_level_commander = HighLevelCommander(cf) # <-- NEW: Initialize HLC

            # Kalman Estimator & PID Controller Setup
            print("Setting estimator and controller...")
            cf.param.set_value('stabilizer.estimator', '2')   # 2 = Kalman estimator
            cf.param.set_value('stabilizer.controller', '1')  # 1 = PID
            time.sleep(0.5)

            # Reset Estimator
            print("Resetting estimator (with timeout)...")
            safe_reset_estimator(scf, timeout=5)
            time.sleep(1)

            # Logging Setup
            log_config = LogConfig(name='RangerPosLog', period_in_ms=100)
            log_config.add_variable('range.front', 'uint16_t')
            log_config.add_variable('range.left', 'uint16_t')
            log_config.add_variable('range.right', 'uint16_t')
            log_config.add_variable('range.back', 'uint16_t')
            log_config.add_variable('range.zrange', 'uint16_t') # downward range
            log_config.add_variable('stateEstimate.x', 'float')
            log_config.add_variable('stateEstimate.y', 'float')

            try:
                cf.log.add_config(log_config)
                log_config.data_received_cb.add_callback(log_callback)
                log_config.start()
                print("Logging started...")
                time.sleep(1) # Give logger time to start

                # Arm
                try:
                    cf.platform.send_arming_request(True)
                except AttributeError:
                    pass

                #Takeoff
                Takeoff(cf, TARGET_HEIGHT_M)

                # Move forward and avoid obstacles till landing pad is detected
                Obstacle_Avoid(cf, TARGET_HEIGHT_M)

                # Land smoothly and relaunch
                Land_or_Relaunch(cf, TARGET_HEIGHT_M)

                # Reset nessesary variables for relaunch
                reset_globals()

                # Move forward and avoid obstacles till landing pad is detected
                Obstacle_Avoid(cf, TARGET_HEIGHT_M)

                # Land smoothly finally
                Land_or_Relaunch(cf, TARGET_HEIGHT_M)

            finally:
                log_config.stop()
                
        cf.commander.send_stop_setpoint()
            
print("Flight sequence")

if __name__ == '__main__':

    main()
