# airplaneSim
Simulation project for aircrafts
- Use JSBSim to Simulate Aircraft Movement
- Extract Sensor Data (GPS, IMU, Airspeed) using pyFMI
- Apply Kalman Filtering for Sensor Fusion
## - 
## Install Required Libraries
You need:

- JSBSim (Flight dynamics engine)
- pyFMI (Sensor simulation)
- pyKalman (Kalman filtering for sensor fusion)
- Matplotlib (Plotting aircraft movement)


## Project Guideline

Here’s a compiled documentation outline for the classes you've selected for version 1 of the library:

---

## **1. Aircraft Class**
### **Overview**
The `Aircraft` class handles all data related to the aircraft's dynamics, control, and flight conditions. It integrates with external aircraft simulation models (such as JSBSim) to simulate realistic behavior. This class is crucial for managing aircraft-specific properties, flight dynamics, and control inputs.

### **Responsibilities**
- **Aircraft Data**: Stores physical parameters such as mass, engine thrust, wingspan, maximum speed, etc.
- **Flight Dynamics**: Manages position, velocity, and attitude (6 degrees of freedom).
- **Control Inputs**: Accepts control inputs (throttle, aileron, rudder, elevator) and updates the aircraft’s state accordingly.
- **Integration**: Interface with external simulation tools (JSBSim, for example) to calculate realistic movement and behavior.

### **Attributes**
- `model_name`: The name or identifier for the aircraft model.
- `mass`: The mass of the aircraft.
- `wingspan`: The wingspan of the aircraft.
- `max_speed`: The maximum speed of the aircraft.
- `engine_thrust`: The maximum engine thrust available.
- `position`: The position of the aircraft in 3D space (latitude, longitude, altitude).
- `velocity`: The velocity of the aircraft (x, y, z components).
- `attitude`: The attitude of the aircraft (pitch, roll, yaw).

### **Methods**
- `update(time_step)`: Updates the aircraft's state based on the time step and input controls.
- `apply_control(throttle, aileron, rudder, elevator)`: Applies control inputs to update the aircraft's behavior.
- `get_state()`: Returns the current state of the aircraft (position, velocity, and attitude).

---

## **2. Mission Planner Class**
### **Overview**
The `Mission Planner` class is designed to store and manage waypoints and flight paths for the aircraft. It enables the simulation of complex flight routes and can be used for planning missions that require navigating between specific geographical points.

### **Responsibilities**
- **Waypoint Management**: Stores a sequence of waypoints (latitude, longitude, altitude).
- **Path Generation**: Provides the flight path for the aircraft, guiding it between waypoints.
- **Mission Planning**: Can include functions to adjust the flight path dynamically or recalculate waypoints.

### **Attributes**
- `waypoints`: A list of waypoints (latitude, longitude, altitude) that define the flight path.
- `current_waypoint`: The current waypoint the aircraft is navigating towards.
- `mission_type`: Type of mission (e.g., test flight, route planning, etc.).

### **Methods**
- `add_waypoint(lat, lon, alt)`: Adds a new waypoint to the flight path.
- `get_next_waypoint()`: Returns the next waypoint for the aircraft to navigate towards.
- `generate_path()`: Creates a continuous flight path based on waypoints, considering flight dynamics.
- `update_waypoints()`: Adjusts the waypoints dynamically, e.g., for emergency rerouting or changes in mission goals.

---

## **3. Kalman Filter Class**
### **Overview**
The `Kalman Filter` class provides a way to fuse sensor data (such as GPS and IMU readings) to produce more accurate estimates of the aircraft’s state (position, velocity, etc.). It combines noisy measurements with the expected model of the system to generate improved estimates of the aircraft's parameters.

### **Responsibilities**
- **Sensor Fusion**: Fuses different sensor inputs, such as GPS, IMU, etc., to produce accurate state estimates.
- **Prediction and Correction**: Implements the prediction and correction steps of the Kalman filter algorithm to improve the accuracy of state estimations.
- **Handling Measurement Noise**: Manages the uncertainty and noise present in the sensor data.

### **Attributes**
- `state_estimate`: Current estimated state (position, velocity, etc.) based on Kalman filter predictions.
- `covariance_matrix`: The error covariance matrix, representing uncertainty in the state estimate.
- `sensor_noise`: The noise characteristics of the sensors involved in the fusion process.

### **Methods**
- `predict()`: Predicts the next state based on the model and previous state estimate.
- `correct(measurement)`: Corrects the state estimate using a new measurement from a sensor.
- `get_state()`: Returns the current estimate of the state (position, velocity, etc.).

---

## **4. Fake Sensor Class**
### **Overview**
The `Fake Sensor` class generates synthetic sensor data for use in simulations, allowing the testing of sensor fusion and aircraft control algorithms without requiring real-world sensor hardware.

### **Responsibilities**
- **Sensor Data Simulation**: Generates fake sensor data for various types of sensors, such as GPS, IMU, and others.
- **Sensor Noise Simulation**: Simulates sensor noise and inaccuracies to make the data more realistic for testing purposes.
- **Integration with Aircraft Model**: Can simulate sensor data based on the aircraft's state, including position, velocity, and attitude.

### **Attributes**
- `sensor_type`: The type of sensor (GPS, IMU, etc.).
- `noise_level`: The level of noise to simulate in the generated data.
- `sensor_state`: The current "state" of the sensor, which could include position, velocity, etc.

### **Methods**
- `generate_data()`: Generates synthetic sensor data for the specified sensor type (GPS, IMU, etc.).
- `apply_noise()`: Adds noise to the generated data based on the specified noise level.
- `get_data()`: Returns the generated sensor data.

---

## **5. Sim Class**
### **Overview**
The `Sim` class is responsible for running the entire simulation, generating data, and plotting or animating the simulation results. It integrates all components (aircraft, mission planner, Kalman filter, fake sensors) to run the simulation and visualize the results.

### **Responsibilities**
- **Simulation Control**: Controls the overall simulation, managing the aircraft's flight, mission planning, and sensor fusion.
- **Data Generation**: Gathers flight data (position, velocity, attitude, sensor readings) during the simulation.
- **Visualization**: Uses Matplotlib (or other libraries) to plot the results and display the aircraft's performance and the sensor fusion outputs.
- **Animation**: Optionally, provides animation of the aircraft's flight along with sensor data overlays.

### **Attributes**
- `aircraft`: An instance of the `Aircraft` class.
- `mission`: An instance of the `Mission Planner` class.
- `kalman_filter`: An instance of the `Kalman Filter` class.
- `sensor`: An instance of the `Fake Sensor` class.
- `time_step`: The simulation time step.
- `simulation_data`: A record of all relevant simulation data collected during the run (e.g., position, velocity, sensor data).

### **Methods**
- `run_simulation()`: Runs the entire simulation, updating aircraft state, collecting sensor data, and applying the Kalman filter.
- `plot_results()`: Plots the results of the simulation, such as position, velocity, and sensor data.
- `animate_simulation()`: Creates an animated visualization of the simulation.
- `save_results()`: Saves simulation data to a file for later analysis.

---

### **Overall Project Flow**
1. **Aircraft**: Simulate aircraft dynamics and controls (e.g., flight path, speed).
2. **Mission Planner**: Set waypoints and generate flight paths.
3. **Fake Sensors**: Generate synthetic sensor data for testing.
4. **Kalman Filter**: Fuse the sensor data for better state estimation.
5. **Sim**: Run the simulation, generate data, and visualize the results.

---
