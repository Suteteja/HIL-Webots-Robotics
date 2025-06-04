# HIL Webots Robotics: ESP32 Path Planning & Control

## 1. Project Overview

This project implements a Hardware-in-the-Loop (HIL) simulation where an ESP32 microcontroller performs path planning using Dijkstra's algorithm. A robot within the Webots simulation environment executes these plans. The system features live visualization of the robot's actions, sensor-based line following, distance sensor-based obstacle detection, and dynamic path re-planning. Communication between the ESP32 (running MicroPython) and the Webots controller (running Python) is achieved via Wi-Fi.

## 2. Purpose

The primary goals of this project are to demonstrate:
* Path planning (Dijkstra's algorithm) on a resource-constrained microcontroller (ESP32).
* Effective Hardware-in-the-Loop (HIL) simulation for robotics development.
* Integration and use of sensors (line sensors, distance sensors) for navigation and obstacle interaction.
* Wireless communication between a simulated environment and an external microcontroller.
* Dynamic obstacle detection by the robot and subsequent path re-planning by the ESP32.
* Live visualization of the robot's state, planned path, and operational environment.

## 3. System Architecture

The system comprises two main Python scripts:

* **`ESP Microcontroller.py` (MicroPython for ESP32):**
    * Connects to a specified Wi-Fi network.
    * Establishes and runs a TCP server to listen for connections from the Webots controller.
    * Maintains an internal grid map representing the environment (pathable areas and obstacles).
    * Receives robot status updates from Webots, including current position, goal, sensor data, and detected obstacles.
    * Updates its internal grid map with newly detected obstacles reported by Webots.
    * Implements Dijkstra's algorithm to calculate the shortest path from the robot's current grid position to a predefined goal.
    * Sends motion commands (e.g., `forward`, `turn_left`, `turn_right`, `stop`) and the complete planned path back to the Webots controller.
    * Handles path re-planning if the goal changes, a new obstacle is detected, or the robot significantly deviates from the current path.

* **`Webots.py` (Python for Webots Robot Controller):**
    * Controls the differential drive robot within the Webots simulation environment.
    * Initializes the robot's hardware components (motors, line sensors, distance sensors).
    * Connects as a TCP client to the server running on the ESP32.
    * Estimates the robot's position and orientation within the world (e.g., using command-based odometry if wheel encoders are not utilized, or encoder-based odometry if implemented).
    * Reads line sensors for line-following behavior and distance sensors for detecting nearby obstacles.
    * Sends comprehensive status updates to the ESP32 (estimated grid position, world pose, line sensor readings, newly detected obstacles).
    * Receives motion commands from the ESP32 and translates them into motor actions.
    * Implements local behaviors such as line centering during forward movement and multi-stage turning routines for precise orientation changes.
    * Uses Matplotlib to generate and update a live visualization of the grid map, robot's current position and trail, the ESP32-planned path, and detected obstacles.

## 4. Features

* **Path Planning:** Dijkstra's algorithm executed on the ESP32.
* **Hardware-in-the-Loop (HIL):** ESP32 microcontroller directly controls a robot simulated in Webots.
* **Wireless Communication:** Utilizes Wi-Fi (TCP/IP) for robust two-way communication between the ESP32 and the computer running Webots.
* **Line Following:** The robot employs ground-facing sensors to detect and follow designated lines on the grid.
* **Obstacle Detection & Avoidance:**
    * The Webots robot uses simulated distance sensors to detect obstacles in its path.
    * Obstacle locations (as grid coordinates) are transmitted to the ESP32.
    * The ESP32 updates its internal map and re-calculates a new path to the goal, avoiding the new obstacle.
* **Live Visualization:** A Matplotlib-based plot displays the robot's environment, its movement trail, the current path planned by the ESP32, and any detected obstacles in real-time.
* **Dynamic Re-planning:** The ESP32 is designed to re-plan paths automatically in response to new obstacles, significant deviation from the planned route, or changes in the goal.

## 5. Setup and Installation

### Dependencies:

* **Python 3.x:** Required on the computer running the Webots simulation.
* **Webots:** Version R2023b or a compatible version.
* **Python Libraries for `Webots.py`:**
    * `matplotlib`: For the live visualization plot. Install using `pip install matplotlib`.
* **MicroPython for ESP32:** The `ESP Microcontroller.py` script is written for the MicroPython environment.
    * Ensure your ESP32 development board is flashed with a recent and stable version of MicroPython.
* **Hardware:**
    * An ESP32 development board.
    * A Wi-Fi network that both the ESP32 and the computer running Webots can connect to.

### Configuration:

1.  **`ESP Microcontroller.py`:**
    * **`WIFI_SSID`**: Set to your Wi-Fi network's name (SSID).
    * **`WIFI_PASSWORD`**: Set to your Wi-Fi network's password.
    * The `grid_map`, `GRID_ROWS`, and `GRID_COLS` variables define the environment's structure and must match the configuration in `Webots.py` and the visual layout in the Webots world file.

2.  **`Webots.py`:**
    * **`ESP32_IP_ADDRESS`**: Update this with the IP address assigned to your ESP32 once it connects to your Wi-Fi network. The ESP32 script typically prints this IP address to its serial console upon successful connection.
    * **`GRID_ROWS`, `GRID_COLS`, `GRID_CELL_SIZE`, `GRID_ORIGIN_X`, `GRID_ORIGIN_Z`**: These parameters define the grid characteristics and the mapping between Webots world coordinates and grid cells. They must be consistent with the ESP32's configuration and the actual dimensions in your Webots world.
    * **`GOAL_ROW`, `GOAL_COL`**: Specify the robot's target destination grid cell.
    * **Robot & Movement Parameters**: Adjust `WHEEL_RADIUS`, `AXLE_LENGTH`, `FORWARD_SPEED`, various turning parameters (e.g., `TURN_SPEED_FACTOR`, `MIN_INITIAL_SPIN_DURATION`), and sensor thresholds (`LINE_THRESHOLD`, `DISTANCE_SENSOR_THRESHOLD`) as needed to fine-tune the robot's movement and sensor interactions for your specific robot model and environment.
    * **`SIMULATED_STATIC_OBSTACLES`**: This list allows you to pre-define static obstacles in the Webots environment for testing purposes.

## 6. How to Run

1.  **Prepare and Flash ESP32:**
    * Open `ESP Microcontroller.py` in a MicroPython-compatible IDE (e.g., Thonny).
    * Ensure your ESP32 board is properly connected to your computer.
    * Upload (flash) the `ESP Microcontroller.py` script to your ESP32.
    * Run the script on the ESP32. Monitor its serial output (e.g., in the Thonny shell) to obtain the IP address the ESP32 acquires after connecting to Wi-Fi. Note this IP address.

2.  **Configure and Prepare Webots:**
    * Open your project in the Webots simulation environment.
    * In the `Webots.py` script, update the `ESP32_IP_ADDRESS` variable with the IP address you noted from the ESP32's serial output.
    * Ensure the robot in your Webots world is assigned `Webots.py` as its controller program.

3.  **Run the Simulation:**
    * Start the simulation in Webots (e.g., by clicking the "Run" or "Play" button).
    * The `Webots.py` script will initialize and attempt to establish a TCP connection to the server running on the ESP32.
    * Upon successful connection, the robot in Webots should start receiving commands from the ESP32 and begin its navigation task.
    * A Matplotlib window should also open, displaying the live visualization of the robot's progress and environment.

**Important Order of Operations:** The `ESP Microcontroller.py` script must be running on the ESP32, connected to Wi-Fi, and its TCP server listening *before* you start the Webots simulation. This ensures that `Webots.py` can successfully connect to the ESP32.

## 7. Code Structure Overview

### `ESP Microcontroller.py` (ESP32)
* **Configuration Section:** Defines Wi-Fi credentials, server port, static grid map, and grid dimensions.
* **Helper Functions:**
    * `connect_wifi()`: Manages the ESP32's connection to the Wi-Fi network.
    * `start_server()`: Initializes and starts the TCP server on the ESP32.
* **Dijkstra's Algorithm:**
    * `SimplePriorityQueue` class: A basic priority queue for Dijkstra.
    * `get_valid_neighbors()`: Finds accessible, pathable neighbors for a given grid cell.
    * `dijkstra()`: Implements the core pathfinding logic.
* **Action Logic:**
    * `get_action_from_path()`: Determines the next robot action (`forward`, `turn_left`, `turn_right`, `stop`) based on the current segment of the planned path and the robot's reported orientation.
* **Main Execution Loop (`if __name__ == "__main__":`)**:
    * Initializes Wi-Fi and the TCP server.
    * Continuously waits for and accepts incoming connections from Webots.
    * In a loop for an active connection:
        * Receives status messages (JSON) from Webots.
        * Parses robot position, goal, sensor data, and detected obstacles.
        * Updates its internal `grid_map` if new obstacles are reported.
        * Triggers path re-planning via Dijkstra if needed (new goal, new obstacle, deviation, periodic replan).
        * Determines the next action and sends the command (JSON) back to Webots.

### `Webots.py` (Webots Controller)
* **Configuration Section:** ESP32 IP/Port, robot physical parameters, grid mapping parameters, goal coordinates, sensor thresholds, and movement/turning parameters.
* **Helper Functions:**
    * `world_to_grid()`: Converts Webots world coordinates to grid cell indices.
    * `grid_to_world_center()`: Converts grid cell indices to Webots world coordinates (center of the cell).
    * `detect_obstacles_from_distance_sensors()`: Interprets distance sensor readings to identify grid cells occupied by obstacles relative to the robot.
    * `update_visualization()`: Creates and continuously updates the Matplotlib plot.
    * `connect_to_esp32_func()`: Manages the TCP client connection to the ESP32 server.
* **Robot Initialization:** Sets up Webots motor devices, ground sensors, and distance sensors.
* **Main Simulation Loop (`while robot.step(timestep) != -1:`):**
    * Reads current values from ground and distance sensors.
    * Performs odometry estimation (command-based, using set motor speeds and time step to estimate displacement).
    * Updates the robot's estimated grid position (`crgp_estimated`).
    * Calls `detect_obstacles_from_distance_sensors()` to find new obstacles.
    * Manages connection to the ESP32 and handles data transmission (sends status) and reception (receives commands).
    * Determines an `effective_command` by considering the command from the ESP32 alongside local sensor-based overrides (e.g., prioritizing line following, initiating a search if off-line but commanded forward).
    * Executes the `effective_command` by setting motor velocities, managing a multi-stage state machine for turning maneuvers (`INITIATE_SPIN`, `SEARCHING_LINE`, `ADJUSTING_ON_LINE`).
    * Periodically calls `update_visualization()`.

## 8. Key Configuration Parameters

* **`ESP Microcontroller.py`:**
    * `WIFI_SSID`, `WIFI_PASSWORD`: Essential for network connectivity.
    * `grid_map`: Defines the initial static layout of the environment. This map is dynamically updated with obstacles detected by Webots.
    * `GRID_ROWS`, `GRID_COLS`: Dimensions of the grid; must be consistent with `Webots.py`.

* **`Webots.py`:**
    * `ESP32_IP_ADDRESS`: Crucial for establishing communication with the ESP32.
    * `GRID_ROWS`, `GRID_COLS`, `GRID_CELL_SIZE`, `GRID_ORIGIN_X`, `GRID_ORIGIN_Z`: These parameters are fundamental for correctly mapping the robot's position between the continuous Webots world and the discrete grid used for path planning. They must match the ESP32's grid settings.
    * `GOAL_ROW`, `GOAL_COL`: Define the robot's target destination on the grid.
    * Movement Parameters (e.g., `FORWARD_SPEED`, `TURN_SPEED_FACTOR`, `MIN_INITIAL_SPIN_DURATION`, `AGGRESSIVE_CORRECTION_DIFFERENTIAL`, `MODERATE_CORRECTION_DIFFERENTIAL`): These values control the robot's speed, turning agility, and line-following responsiveness. Fine-tuning these is often necessary for optimal performance.
    * Sensor Thresholds (`LINE_THRESHOLD`, `DISTANCE_SENSOR_THRESHOLD`): These determine the sensitivity of the line and distance sensors. Proper calibration is key for reliable line detection and obstacle avoidance.
    * `SIMULATED_STATIC_OBSTACLES`: A list to define fixed obstacles directly in the Webots script for testing the obstacle avoidance logic.

## 9. Authors / Credits
* Albert Jestin Sutedja (Student ID: 466092)
* Inspiration and assistance from Simon for code design concepts.
* ESP32 connection design based on concepts from Simon.
