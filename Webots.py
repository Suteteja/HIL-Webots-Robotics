from controller import Robot, DistanceSensor, Motor
import socket
import time
import math
import matplotlib.pyplot as plt
import json

# Network Configuration
ESP32_IP_ADDRESS = "192.168.178.178" # Replace with ESP32 IP
ESP32_PORT = 8080

# Parameters are in meters
# Robot Parameters
WHEEL_RADIUS = 0.0205  
AXLE_LENGTH = 0.057    

# Grid Configuration
GRID_ROWS = 15
GRID_COLS = 21
GRID_CELL_SIZE = 0.051 

GRID_ORIGIN_X = 0.050002 
GRID_ORIGIN_Z = -0.639e-05 

GOAL_ROW = 14
GOAL_COL = 0

# Parameters
FORWARD_SPEED = 2.5 # rad/s
LINE_THRESHOLD = 600 # ground sensor line threshold

# Distance Sensor Parameters
DISTANCE_SENSOR_THRESHOLD = 300
OBSTACLE_DETECTION_ENABLED = True # off to bypass

# Time unit in seconds
# Turning Parameters
TURN_SPEED_FACTOR = 1.2
MIN_INITIAL_SPIN_DURATION = 0.8 
MAX_SEARCH_SPIN_DURATION = 20.0 
MAX_ADJUST_DURATION = 5.0   
TURN_ADJUST_BASE_SPEED = FORWARD_SPEED * 0.8
TURN_UNTIL_LINE_FOUND = True

# Line Centering Parameters
AGGRESSIVE_CORRECTION_DIFFERENTIAL = FORWARD_SPEED * 1.0
MODERATE_CORRECTION_DIFFERENTIAL = FORWARD_SPEED * 0.8

# World grid definition
world_grid = [
    [1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,1,0,1,0,1,0],
    [1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,1,0,1,0,1,0],
    [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    [0,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,0],
    [0,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,0],
    [0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,0],
    [0,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,0],
    [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    [0,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,0],
    [0,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0],
    [0,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,0],
    [0,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,0],
    [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    [0,1,0,1,0,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
    [0,1,0,1,0,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1]
]

SIMULATED_STATIC_OBSTACLES = [(5, 10)]
detected_obstacles_grid = set()
recent_new_obstacles = []

for obs_row, obs_col in SIMULATED_STATIC_OBSTACLES:
    if 0 <= obs_row < GRID_ROWS and 0 <= obs_col < GRID_COLS:
        world_grid[obs_row][obs_col] = 1
        detected_obstacles_grid.add((obs_row, obs_col))
        recent_new_obstacles.append((obs_row, obs_col))
        print(f"INFO: Simulated static obstacle added at ({obs_row}, {obs_col})")

plt.ion()
fig, ax = None, None # Initialise plot 
robot_trail_world = []
planned_path_grid = []
webots_internal_turn_phase = 'NONE'
webots_turn_command_active = None
turn_phase_start_time = 0.0

# --- Helper Functions ---
def world_to_grid(world_x, world_z):
    col = round((world_x - GRID_ORIGIN_X) / GRID_CELL_SIZE)
    row = round((world_z - GRID_ORIGIN_Z) / GRID_CELL_SIZE)
    return max(0, min(row, GRID_ROWS - 1)), max(0, min(col, GRID_COLS - 1))

def grid_to_world_center(row, col):
    return GRID_ORIGIN_X + col * GRID_CELL_SIZE, GRID_ORIGIN_Z + row * GRID_CELL_SIZE

def get_line_centered_position(rwp_val, crgp_val, ldf_val): # Renamed to avoid conflict
    return rwp_val['x'], rwp_val['z']

def detect_obstacles_from_distance_sensors(rwp_val, robot_theta, distance_values):
    if not OBSTACLE_DETECTION_ENABLED: return []
    newly_detected = []
    current_row, current_col = world_to_grid(rwp_val['x'], rwp_val['z'])
    theta_deg_norm = math.degrees(robot_theta % (2 * math.pi))
    if theta_deg_norm < 0: theta_deg_norm += 360

    obstacle_relative_positions = {}
    if (315 <= theta_deg_norm <= 360) or (0 <= theta_deg_norm <= 45): # RIGHT
        obstacle_relative_positions = {0: (0, 1), 1: (-1, 1), 2: (1, 1)}
    elif 45 < theta_deg_norm <= 135: # DOWN
        obstacle_relative_positions = {0: (1, 0), 1: (1, 1), 2: (1, -1)}
    elif 135 < theta_deg_norm <= 225: # LEFT
        obstacle_relative_positions = {0: (0, -1), 1: (1, -1), 2: (-1, -1)}
    elif 225 < theta_deg_norm <= 315: # UP
        obstacle_relative_positions = {0: (-1, 0), 1: (-1, -1), 2: (-1, 1)}

    for i, dist_val in enumerate(distance_values):
        if dist_val > DISTANCE_SENSOR_THRESHOLD:
            if i in obstacle_relative_positions:
                dr, dc = obstacle_relative_positions[i]
                obs_r, obs_c = current_row + dr, current_col + dc
                if 0 <= obs_r < GRID_ROWS and 0 <= obs_c < GRID_COLS:
                    if (obs_r, obs_c) not in detected_obstacles_grid:
                        newly_detected.append((obs_r, obs_c))
                        detected_obstacles_grid.add((obs_r, obs_c))
    return newly_detected

def update_visualization(rwp_val, crgp_val, path_esp): # Renamed to avoid conflict
    global fig, ax, robot_trail_world
    if fig is None:
        fig, ax = plt.subplots(figsize=(12, 9))
        ax.set_aspect('equal'); ax.set_title('HIL Navigation (Command-Based Odometry)', fontsize=14, fontweight='bold')
        ax.set_xlabel('World X (m)'); ax.set_ylabel('World Z (m)')
        for r_idx in range(GRID_ROWS + 1): ax.plot([GRID_ORIGIN_X, GRID_ORIGIN_X + GRID_COLS * GRID_CELL_SIZE], [GRID_ORIGIN_Z + r_idx * GRID_CELL_SIZE]*2, 'k-', alpha=0.2, lw=0.5)
        for c_idx in range(GRID_COLS + 1): ax.plot([GRID_ORIGIN_X + c_idx * GRID_CELL_SIZE]*2, [GRID_ORIGIN_Z, GRID_ORIGIN_Z + GRID_ROWS * GRID_CELL_SIZE], 'k-', alpha=0.2, lw=0.5)
        margin = GRID_CELL_SIZE * 2; ax.set_xlim(GRID_ORIGIN_X - margin, GRID_ORIGIN_X + GRID_COLS * GRID_CELL_SIZE + margin); ax.set_ylim(GRID_ORIGIN_Z - margin, GRID_ORIGIN_Z + GRID_ROWS * GRID_CELL_SIZE + margin)
        from matplotlib.patches import Patch
        legend_elements = [Patch(fc='black',label='Pathable'), Patch(fc='lightgrey',label='Obstacle (Grid)'), Patch(fc='red',label='Detected Obstacle'), plt.Line2D([0],[0],color='cyan',lw=2,label='Trail'), plt.Line2D([0],[0],color='magenta',marker='o',ls='--',label='Path (ESP)'), plt.Line2D([0],[0],color='red',marker='o',ls='',label='Robot'), plt.Line2D([0],[0],color='green',marker='*',ls='',label='Goal')]
        ax.legend(handles=legend_elements, loc='upper left', bbox_to_anchor=(1.02,1)); plt.tight_layout(); plt.show(block=False); plt.pause(0.01)

    for patch_item in ax.patches[:]: patch_item.remove()
    num_static_lines = (GRID_ROWS + 1) + (GRID_COLS + 1)
    for line_item in ax.lines[num_static_lines:]: line_item.remove()

    for r in range(GRID_ROWS):
        for c in range(GRID_COLS):
            cx, cz = grid_to_world_center(r,c)
            is_det_obs = (r,c) in detected_obstacles_grid
            clr, alp = ('red',0.7) if is_det_obs else (('black',0.6) if world_grid[r][c]==0 else ('lightgrey',0.3))
            ax.add_patch(plt.Rectangle((cx-GRID_CELL_SIZE/2, cz-GRID_CELL_SIZE/2), GRID_CELL_SIZE,GRID_CELL_SIZE, fc=clr,alpha=alp,ec='gray',lw=0.5))

    try: raw_gs_vals = [s.getValue() for s in gs_wb]; line_det = [1 if v<LINE_THRESHOLD else 0 for v in raw_gs_vals]
    except: line_det,raw_gs_vals = [0,0,0],[0,0,0]
    
    disp_x,disp_z = get_line_centered_position(rwp_val,crgp_val,line_det)
    robot_trail_world.append((disp_x,disp_z));
    if len(robot_trail_world)>200: robot_trail_world.pop(0)
    if len(robot_trail_world)>1: trail_x,trail_z=zip(*robot_trail_world); ax.plot(trail_x,trail_z,'cyan',lw=2,alpha=0.7)
    
    if path_esp and len(path_esp)>1:
        path_w = [grid_to_world_center(r,c) for r,c in path_esp]
        if path_w: p_x,p_z=zip(*path_w); ax.plot(p_x,p_z,'mo--',lw=2,ms=5,alpha=0.8)
    ax.plot(disp_x,disp_z,'ro',ms=10,mec='darkred',mew=1)
    arr_len=GRID_CELL_SIZE*0.7; dx,dz = arr_len*math.cos(rwp_val['theta']), arr_len*math.sin(rwp_val['theta'])
    ax.add_patch(plt.matplotlib.patches.FancyArrowPatch((disp_x,disp_z),(disp_x+dx,disp_z+dz),arrowstyle='->',mutation_scale=15,color='darkred',lw=2))
    
    if crgp_val:
        cx,cz=grid_to_world_center(crgp_val[0],crgp_val[1]); sensors_on=any(line_det)
        hl_clr,hl_alp = ('green',0.4) if sensors_on else ('yellow',0.4)
        ax.add_patch(plt.Rectangle((cx-GRID_CELL_SIZE/2,cz-GRID_CELL_SIZE/2),GRID_CELL_SIZE,GRID_CELL_SIZE,ec=hl_clr,fc='none',alpha=1.0,lw=3,ls='--'))
    goal_x,goal_z=grid_to_world_center(GOAL_ROW,GOAL_COL); ax.plot(goal_x,goal_z,'g*',ms=15,mec='darkgreen',mew=1.5)
    
    if hasattr(ax, 'info_panel_text_obj') and ax.info_panel_text_obj in ax.texts: ax.info_panel_text_obj.remove()
    info_txt = (f"Grid:{crgp_val} G:({GOAL_ROW},{GOAL_COL})\nLine:{'ON' if any(line_det) else 'OFF'} {line_det}\n"
                f"Obs:{len(detected_obstacles_grid)} Turn:{webots_internal_turn_phase}\n"
                f"Est.W:X={rwp_val['x']:.2f},Z={rwp_val['z']:.2f},Θ={math.degrees(rwp_val['theta']):.0f}°")
    ax.info_panel_text_obj = ax.text(0.02,0.98,info_txt,transform=ax.transAxes,va='top',fontsize=8,bbox=dict(boxstyle='round,pad=0.4',facecolor='lightblue',alpha=0.8))
    plt.draw(); plt.pause(0.001)

# --- Robot Setup ---
robot = Robot()
timestep = int(robot.getBasicTimeStep())
time_step_sec = timestep / 1000.0 # Timestep in seconds

rwp_estimate = {'x': 0.0, 'z': 0.0, 'theta': 0.0} # Robot World Pose (ESTIMATED)

left_motor = robot.getDevice('left wheel motor'); right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf')); right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0); right_motor.setVelocity(0.0)

# Encoders are NOT used for odometry in this version
# left_encoder = robot.getDevice('left wheel sensor'); right_encoder = robot.getDevice('right wheel sensor')
# left_encoder.enable(timestep); right_encoder.enable(timestep)

gs_wb = [robot.getDevice(name) for name in ['gs0', 'gs1', 'gs2']]
for sensor in gs_wb: sensor.enable(timestep)

ps_all = [robot.getDevice(f'ps{i}') for i in range(8)]
for sensor in ps_all: sensor.enable(timestep)
distance_sensors_selected = [ps_all[0], ps_all[7], ps_all[5]] # F, FL, FR

client_socket = None; is_connected = False; esp32_command = 'stop'

def connect_to_esp32_func(): # Renamed to avoid conflict
    global client_socket, is_connected
    print(f"Connecting to ESP32 ({ESP32_IP_ADDRESS}:{ESP32_PORT})...")
    try:
        if client_socket: client_socket.close()
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.settimeout(2.0); client_socket.connect((ESP32_IP_ADDRESS, ESP32_PORT))
        client_socket.settimeout(0.05); is_connected = True; print("✅ ESP32 Connected.")
    except Exception as e: print(f"❌ ESP32 Conn Fail: {e}"); is_connected=False; client_socket=None

INITIAL_GRID_ROW, INITIAL_GRID_COL = 2, 20
rwp_estimate['x'], rwp_estimate['z'] = grid_to_world_center(INITIAL_GRID_ROW, INITIAL_GRID_COL)
rwp_estimate['theta'] = math.pi # Start West
crgp_estimated = world_to_grid(rwp_estimate['x'], rwp_estimate['z'])

print(f"Robot init @ grid {crgp_estimated}, est.world ({rwp_estimate['x']:.2f},{rwp_estimate['z']:.2f}), Θ={math.degrees(rwp_estimate['theta']):.0f}°")
print(f"Goal: ({GOAL_ROW},{GOAL_COL}). Obstacle Det: {'ON' if OBSTACLE_DETECTION_ENABLED else 'OFF'}")
print("-" * 60)

iteration, last_conn_attempt, last_data_send_time, last_obs_check_time = 0,0,0,0
current_left_speed, current_right_speed = 0.0, 0.0 # Store current motor speeds

# --- Main Loop ---
while robot.step(timestep) != -1:
    if iteration == 0: connect_to_esp32_func(); update_visualization(rwp_estimate, crgp_estimated, planned_path_grid)
    iteration += 1; current_sim_time = robot.getTime()

    # --- Sensor Readings ---
    raw_gs_values = [s.getValue() for s in gs_wb]
    line_detected = [1 if v < LINE_THRESHOLD else 0 for v in raw_gs_values]
    left_gs, center_gs, right_gs = line_detected
    
    ps_values_all = [p.getValue() for p in ps_all]
    current_dist_sens_vals = [ps_values_all[0], ps_values_all[7], ps_values_all[5]]

    # Linear speed of each wheel (m/s)
    v_left = current_left_speed * WHEEL_RADIUS
    v_right = current_right_speed * WHEEL_RADIUS

    # Linear and angular velocity of the robot
    linear_velocity = (v_left + v_right) / 2.0
    angular_velocity = (v_right - v_left) / AXLE_LENGTH

    # Update pose
    rwp_estimate['theta'] += angular_velocity * time_step_sec
    # Normalize theta to -pi to pi
    rwp_estimate['theta'] = math.atan2(math.sin(rwp_estimate['theta']), math.cos(rwp_estimate['theta']))
    
    # Average orientation over the timestep for displacement calculation
    avg_theta = rwp_estimate['theta'] - (angular_velocity * time_step_sec / 2.0)
    rwp_estimate['x'] += linear_velocity * math.cos(avg_theta) * time_step_sec
    rwp_estimate['z'] += linear_velocity * math.sin(avg_theta) * time_step_sec
    
    new_crgp_est = world_to_grid(rwp_estimate['x'], rwp_estimate['z'])
    if new_crgp_est != crgp_estimated: crgp_estimated = new_crgp_est

    # --- Obstacle Detection ---
    if current_sim_time - last_obs_check_time > 0.2:
        if OBSTACLE_DETECTION_ENABLED:
            new_obs = detect_obstacles_from_distance_sensors(rwp_estimate, rwp_estimate['theta'], current_dist_sens_vals)
            if new_obs: print(f"🚧 {len(new_obs)} new obstacles by sensors!"); recent_new_obstacles.extend(new_obs)
        last_obs_check_time = current_sim_time
    
    # --- ESP Connection & Communication ---
    if not is_connected:
        if current_sim_time - last_conn_attempt > 3.0: connect_to_esp32_func(); last_conn_attempt = current_sim_time
        current_left_speed, current_right_speed = 0.0, 0.0 # Prepare for next cycle's estimation
        left_motor.setVelocity(0.0); right_motor.setVelocity(0.0)
        if iteration % 10 == 0: update_visualization(rwp_estimate, crgp_estimated, planned_path_grid)
        continue

    if current_sim_time - last_data_send_time > 0.1:
        try:
            status_data = {'type':'webots_status','robot_grid_pos':list(crgp_estimated), 'goal_grid_pos':[GOAL_ROW,GOAL_COL],
                           'world_pose':{'x':round(rwp_estimate['x'],3),'z':round(rwp_estimate['z'],3),'theta_rad':round(rwp_estimate['theta'],3)},
                           'sensors_binary':line_detected, 'detected_obstacles':recent_new_obstacles.copy()}
            client_socket.sendall((json.dumps(status_data) + '\n').encode('utf-8'))
            last_data_send_time = current_sim_time
            if recent_new_obstacles: recent_new_obstacles.clear()
        except Exception as e: print(f"Send Err: {e}"); is_connected=False; client_socket.close(); client_socket=None; continue
    
    try:
        resp = client_socket.recv(1024)
        if resp:
            dec_resps = resp.decode('utf-8').strip().split('\n')
            for msg in dec_resps:
                if not msg.strip(): continue
                try:
                    esp_rcv_data = json.loads(msg)
                    if esp_rcv_data.get('type') == 'esp32_command':
                        new_cmd = esp_rcv_data.get('action','stop')
                        if new_cmd!=esp32_command and esp32_command in ['turn_left','turn_right'] and new_cmd not in ['turn_left','turn_right']:
                            webots_internal_turn_phase = 'NONE'; webots_turn_command_active=None
                        esp32_command = new_cmd
                        planned_path_grid = esp_rcv_data.get('path', planned_path_grid)
                except json.JSONDecodeError as e: print(f"JSON Err ESP: {e} (Msg:'{msg}')")
                except Exception as e: print(f"Proc ESP Err: {e}")
    except socket.timeout: pass
    except Exception as e: print(f"Recv Err: {e}"); is_connected=False; client_socket.close(); client_socket=None; continue

 # --- Determine Effective Command ---
    effective_command = esp32_command  # Default to ESP32 command
    sensors_on_line = any(line_detected)

    # 1. Handle ongoing Webots-internal turns
    if webots_internal_turn_phase != 'NONE':
        if esp32_command == 'stop': # ESP32 can interrupt an internal turn with 'stop'
            effective_command = 'stop'
            webots_internal_turn_phase = 'NONE'
            webots_turn_command_active = None
        else: # Otherwise, Webots continues its current turn
            effective_command = webots_turn_command_active
    # 2. Sensor overrides if not in an internal turn
    elif sensors_on_line and esp32_command not in ['turn_left', 'turn_right', 'stop']:
        effective_command = 'forward' # Override: Follow line if ESP isn't turning/stopping
    elif not sensors_on_line and esp32_command == 'forward':
        effective_command = 'turn_left' # Override: Search for line if ESP says forward but none detected
    
    # 3. Specific override: If ESP32 commanded 'stop', but robot is off-line and not already turning to search
    if esp32_command == 'stop' and not sensors_on_line and effective_command == 'stop':
        print(f"INFO: ESP 'stop', off-line. Webots overriding to 'turn_left'.")
        effective_command = 'turn_left'

    # --- Execute Command & Set Motor Speeds ---
    next_left_speed, next_right_speed = 0.0, 0.0

    if effective_command == 'stop':
        next_left_speed, next_right_speed = 0.0, 0.0
        webots_internal_turn_phase = 'NONE'
        webots_turn_command_active = None

    elif effective_command == 'forward':
        webots_internal_turn_phase = 'NONE' # Stop any ongoing turn if commanded forward
        webots_turn_command_active = None
        base_s = FORWARD_SPEED
        # Line following logic, change parameter on top
        if not left_gs and center_gs and not right_gs: next_left_speed,next_right_speed = base_s,base_s
        elif left_gs and center_gs and not right_gs: next_left_speed,next_right_speed = base_s-MODERATE_CORRECTION_DIFFERENTIAL,base_s
        elif not left_gs and center_gs and right_gs: next_left_speed,next_right_speed = base_s,base_s-MODERATE_CORRECTION_DIFFERENTIAL
        elif left_gs and not center_gs and not right_gs: next_left_speed,next_right_speed = base_s-AGGRESSIVE_CORRECTION_DIFFERENTIAL,base_s
        elif not left_gs and not center_gs and right_gs: next_left_speed,next_right_speed = base_s,base_s-AGGRESSIVE_CORRECTION_DIFFERENTIAL
        elif left_gs and center_gs and right_gs: next_left_speed,next_right_speed = base_s*0.7,base_s*0.7
        elif not sensors_on_line: next_left_speed,next_right_speed = base_s*0.2,base_s*0.2 # Lost line
        else: next_left_speed,next_right_speed = base_s*0.3,base_s*0.3 # Other cases (e.g., only one side sensor on)

    elif effective_command in ['turn_left', 'turn_right']:
        current_turn_cmd = effective_command
        # Initiate or continue turn sequence
        if webots_turn_command_active != current_turn_cmd or webots_internal_turn_phase == 'NONE':
            webots_turn_command_active = current_turn_cmd
            webots_internal_turn_phase = 'INITIATE_SPIN'
            turn_phase_start_time = current_sim_time
            print(f"Turn {webots_turn_command_active} initiated.")

        # Handle turn phases
        if webots_internal_turn_phase == 'INITIATE_SPIN':
            s_i, s_o = -FORWARD_SPEED * TURN_SPEED_FACTOR * 0.8, FORWARD_SPEED * TURN_SPEED_FACTOR * 1.1
            next_left_speed, next_right_speed = (s_i, s_o) if webots_turn_command_active == 'turn_left' else (s_o, s_i)
            if current_sim_time - turn_phase_start_time > MIN_INITIAL_SPIN_DURATION:
                webots_internal_turn_phase = 'SEARCHING_LINE'
                turn_phase_start_time = current_sim_time
                print(f"🔍 Search for {webots_turn_command_active}")
        elif webots_internal_turn_phase == 'SEARCHING_LINE':
            s_i, s_o = -FORWARD_SPEED * TURN_SPEED_FACTOR * 0.5, FORWARD_SPEED * TURN_SPEED_FACTOR * 0.9
            next_left_speed, next_right_speed = (s_i, s_o) if webots_turn_command_active == 'turn_left' else (s_o, s_i)
            if sensors_on_line:
                webots_internal_turn_phase = 'ADJUSTING_ON_LINE'
                turn_phase_start_time = current_sim_time
                print(f"✅ Line acquired {webots_turn_command_active}")
            elif not TURN_UNTIL_LINE_FOUND and current_sim_time - turn_phase_start_time > MAX_SEARCH_SPIN_DURATION:
                print(f"⏰ Search Timeout {webots_turn_command_active}")
                webots_internal_turn_phase = 'NONE'
                next_left_speed, next_right_speed = 0,0
        elif webots_internal_turn_phase == 'ADJUSTING_ON_LINE':
            b = TURN_ADJUST_BASE_SPEED
            m_d = MODERATE_CORRECTION_DIFFERENTIAL * (b / FORWARD_SPEED)
            a_d = AGGRESSIVE_CORRECTION_DIFFERENTIAL * (b / FORWARD_SPEED)
            if not left_gs and center_gs and not right_gs: # Perfect center
                next_left_speed, next_right_speed = b * 0.3, b * 0.3
                webots_internal_turn_phase = 'NONE'
                webots_turn_command_active = None
            elif left_gs and center_gs and not right_gs: next_left_speed,next_right_speed = b-m_d,b
            elif not left_gs and center_gs and right_gs: next_left_speed,next_right_speed = b,b-m_d
            elif left_gs and not center_gs and not right_gs: next_left_speed,next_right_speed = b-a_d,b
            elif not left_gs and not center_gs and right_gs: next_left_speed,next_right_speed = b,b-a_d
            elif not sensors_on_line: # Line lost during adjustment
                print(f"❌ Line lost during adjust. Re-search.")
                webots_internal_turn_phase = 'SEARCHING_LINE'
                turn_phase_start_time = current_sim_time
                # Speeds for searching will be set in the next iteration by SEARCHING_LINE phase
            else: # Other cases on line (e.g., all sensors on)
                next_left_speed, next_right_speed = b * 0.7, b * 0.7
            
            if current_sim_time - turn_phase_start_time > MAX_ADJUST_DURATION and webots_internal_turn_phase == 'ADJUSTING_ON_LINE': # Check phase again
                print(f"⏰ Adjust Timeout {webots_turn_command_active}")
                webots_internal_turn_phase = 'NONE'
                webots_turn_command_active = None
                next_left_speed,next_right_speed=0,0
    
    # Apply motor velocities
    left_motor.setVelocity(next_left_speed)
    right_motor.setVelocity(next_right_speed)
    current_left_speed, current_right_speed = next_left_speed, next_right_speed # Store for next step's odometry estimation

    # --- Visualization & Logging ---
    if iteration % 3 == 0: update_visualization(rwp_estimate, crgp_estimated, planned_path_grid)
    if iteration % 25 == 0:
        conn_stat = ("Yes" if is_connected else "No")
        line_s = ("ON" if sensors_on_line else "OFF")
        obs_s = f"Obst:{len(detected_obstacles_grid)}" if detected_obstacles_grid else "Null"
        turn_s = f"T:{webots_internal_turn_phase}" if webots_internal_turn_phase!='NONE' else ""
        cmd_d = f"{esp32_command}" if esp32_command==effective_command else f"{esp32_command}→{effective_command}"
        print(f"T:{current_sim_time:.1f}|ESP:{conn_stat}|Cmd:{cmd_d}|Grid:{crgp_estimated}|L:{line_s}{line_detected}|{obs_s}|{turn_s}")

# --- Cleanup ---
if client_socket:
    try: client_socket.close()
    except: pass
if fig: print("\nSim End."); plt.ioff(); plt.show(block=True)
print("Ctrl Finish.")
