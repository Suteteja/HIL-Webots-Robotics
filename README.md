# HIL-Webots-Robotics


 # 1. Project Overview

This project implements a Hardware-in-the-Loop (HIL) simulation where an ESP32 microcontroller performs path planning using Dijkstra's algorithm, and a robot in the Webots simulation environment executes these plans. The system features live visualization, sensor-based line following, distance sensor-based obstacle detection, and dynamic path re-planning. Communication between the ESP32 and the Webots controller is achieved via Wi-Fi.

# 2. Purpose

The primary goal is to demonstrate:
* Path planning (Dijkstra's algorithm) on a resource-constrained microcontroller (ESP32).
* Hardware-in-the-Loop simulation for robotics development.
* Integration of sensors (line sensors, distance sensors) for navigation and obstacle interaction.
* Wireless communication between a simulated environment and an external microcontroller.
* Dynamic obstacle detection and path re-planning.
* Live visualization of the robot's state, path, and environment.

# 3. System Architecture

The system consists of two main Python scripts:

* **`ESP Microcontroller.py`:**
    * Connects to a Wi-Fi network.
    * Runs a TCP server to listen for connections from the Webots controller.
    * Maintains a grid map of the environment.
    * Receives robot status (current position, goal, sensor data, detected obstacles) from Webots.
    * Updates its internal grid map with newly detected obstacles.
    * Implements Dijkstra's algorithm to calculate the shortest path from the robot's current position to a predefined goal.
    * Sends motion commands (`forward`, `turn_left`, `turn_right`, `stop`) and the planned path back to Webots.
    * Handles path re-planning if the goal changes, an obstacle is detected, or the robot deviates.

* **`Webots.py` (Python for Webots Robot Controller):**
    * Controls the robot within the Webots simulation environment.
    * Initializes robot hardware (motors, line sensors, distance sensors).
    * Connects as a TCP client to the ESP32 server.
    * Estimates the robot's position and orientation (using command-based odometry if encoders are not used, or encoder-based odometry if implemented).
    * Reads line sensors for line following and distance sensors for obstacle detection.
    * Sends robot status (estimated grid position, world pose, line sensor readings, newly detected obstacles) to the ESP32.
    * Receives motion commands from the ESP32 and executes them.
    * Implements local behaviors like line centering and multi-stage turning routines.
    * Uses Matplotlib to provide a live visualization of the grid, robot, planned path, and detected obstacles.

## 4. Features

* **Path Planning:** Dijkstra's algorithm on ESP32.
* **Hardware-in-the-Loop (HIL):** ESP32 controls a Webots simulated robot.
* **Wireless Communication:** Wi-Fi based TCP/IP communication.
* **Line Following:** Robot uses ground sensors to follow lines.
* **Obstacle Detection:** Uses simulated distance sensors.
    * Webots detects obstacles and sends their locations to the ESP32.
    * ESP32 updates its map and re-plans the path.
* **Live Visualization:** Matplotlib plot showing the robot's environment, trail, planned path, and detected obstacles.
* **Dynamic Re-planning:** ESP32 re-plans if obstacles are found or the robot deviates.

## 5. Setup and Installation

### Dependencies:

* **Python 3.x** on the computer running Webots.
* **Webots:** Version R2023b or compatible.
* **Python Libraries for `Webots.py`:**
    * `matplotlib`: For live visualization. Install using `pip install matplotlib`.
* **MicroPython for ESP32:** The `ESP Djickstra.py` script is written for MicroPython.
    * Ensure your ESP32 is flashed with a recent version of MicroPython.
* **Hardware:**
    * An ESP32 development board.
    * A Wi-Fi network accessible by both the ESP32 and the computer running Webots.

### Configuration:

1.  **`ESP Djickstra.py`:**
    * Update `WIFI_SSID` with your Wi-Fi network name.
    * Update `WIFI_PASSWORD` with your Wi-Fi password.
    * The `grid_map`, `GRID_ROWS`, and `GRID_COLS` should match the Webots environment if you modify it.

2.  **`Webots.py`:**
    * Update `ESP32_IP_ADDRESS` with the IP address assigned to your ESP32 after it connects to Wi-Fi (the ESP32 script prints this to its serial console).
    * `GRID_ROWS`, `GRID_COLS`, `GRID_CELL_SIZE`, `GRID_ORIGIN_X`, `GRID_ORIGIN_Z` must match the Webots world setup and the ESP32's grid configuration.
    * `GOAL_ROW`, `GOAL_COL` define the target destination for the robot.
    * Adjust robot parameters (`WHEEL_RADIUS`, `AXLE_LENGTH`), sensor thresholds (`LINE_THRESHOLD`, `DISTANCE_SENSOR_THRESHOLD`), and movement speeds (`FORWARD_SPEED`, turning parameters) as needed for your specific robot model or desired behavior.

## 6. How to Run

1.  **Flash ESP32:**
    * Open `ESP Djickstra.py` in a MicroPython-compatible IDE (e.g., Thonny).
    * Ensure your ESP32 is connected to your computer.
    * Upload the script to your ESP32.
    * Run the script on the ESP32. Monitor its serial output to get the IP address it acquires after connecting to Wi-Fi.

2.  **Configure Webots Controller:**
    * Open your Webots project.
    * Ensure the robot controller is set to run `Webots.py`.
    * Update `ESP32_IP_ADDRESS` in `Webots.py` with the IP address obtained from the ESP32 in the previous step.

3.  **Run Simulation:**
    * Start the Webots simulation.
    * The `Webots.py` script will attempt to connect to the ESP32 server.
    * Once connected, the robot should start receiving commands from the ESP32 and begin navigation.
    * A Matplotlib window should appear showing the live visualization.

**Order of Operations:** The ESP32 server script (`ESP Djickstra.py`) must be running and connected to Wi-Fi *before* you start the Webots simulation, so that `Webots.py` can successfully connect to it.

## 7. Code Structure

### `ESP Djickstra.py`
* **Configuration:** Wi-Fi credentials, server port, grid map.
* **`connect_wifi()`:** Handles Wi-Fi connection.
* **`start_server()`:** Sets up the TCP server.
* **`SimplePriorityQueue`, `get_valid_neighbors`, `dijkstra()`:** Implementation of Dijkstra's algorithm.
* **`get_action_from_path()`:** Determines robot actions based on the current path and robot orientation.
* **Main Loop (`if __name__ == "__main__":`)**:
    * Initializes WiFi and server.
    * Listens for client connections (from Webots).
    * Receives status messages from Webots.
    * Updates its internal `grid_map` with obstacles reported by Webots.
    * Triggers path re-planning (using Dijkstra) when necessary.
    * Sends commands back to Webots.

### `Webots.py`
* **Configuration:** ESP32 IP/Port, robot parameters, grid parameters, goal, sensor/movement thresholds.
* **Helper Functions:** `world_to_grid`, `grid_to_world_center`.
* **`detect_obstacles_from_distance_sensors()`:** Processes distance sensor data to identify obstacle locations.
* **`update_visualization()`:** Manages the live Matplotlib plot.
* **`connect_to_esp32_func()`:** Handles TCP client connection to the ESP32.
* **Robot Initialization:** Sets up motors, sensors.
* **Main Loop (`while robot.step(timestep) != -1:`):**
    * Reads sensors (ground, distance).
    * Performs odometry estimation (command-based if encoders aren't used).
    * Updates robot's grid position.
    * Calls obstacle detection.
    * Communicates with ESP32 (sends status, receives commands).
    * Determines `effective_command` based on ESP32 command and local sensor overrides (e.g., line following, emergency stop if lost).
    * Executes motor commands based on `effective_command`, including multi-stage turning logic.
    * Updates the visualization.

## 8. Key Configuration Parameters

* **`ESP Djickstra.py`:**
    * `WIFI_SSID`, `WIFI_PASSWORD`: Your Wi-Fi network details.
    * `grid_map`: Defines the static layout of the environment and will be updated with dynamic obstacles.
    * `GRID_ROWS`, `GRID_COLS`: Must match `Webots.py` and the actual world.
* **`Webots.py`:**
    * `ESP32_IP_ADDRESS`: IP address of the ESP32.
    * `GRID_ROWS`, `GRID_COLS`, `GRID_CELL_SIZE`, `GRID_ORIGIN_X`, `GRID_ORIGIN_Z`: Define the mapping between world coordinates and the grid. Must match ESP32.
    * `GOAL_ROW`, `GOAL_COL`: The robot's destination.
    * `FORWARD_SPEED`, `TURN_SPEED_FACTOR`, `MIN_INITIAL_SPIN_DURATION`, etc.: Control robot movement speeds and turning behavior.
    * `LINE_THRESHOLD`, `DISTANCE_SENSOR_THRESHOLD`: Critical for sensor interpretation.
    * `SIMULATED_STATIC_OBSTACLES`: Allows pre-defining obstacles in Webots for testing.

## 9. Authors / Credits
* Albert Jestin Sutedja (Student ID: 466092)
* Inspiration and assistance from Simon for code design concepts.
* ESP32 connection design based on concepts from Simon.
