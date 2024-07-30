## `sensors` Directory

The `sensors` directory houses the various sensor simulation modules used in the UAV and VTOL aircraft simulation. Each sensor class is designed to simulate data for its specific kind of sensor, and publishes the simulated data to a respective ROS topic. The frequency of these updates is defined by the time interval set in each sensor's constructor. 

The following sensors are simulated:

1. **AttitudeSensor**: The `AttitudeSensor` is a sensor class that periodically publishes the UAV's attitude, represented as a quaternion, to a specified ROS topic.

2. **PressureSensor**: The `PressureSensor` and `TemperatureSensor` are sensor classes that periodically publish the UAV's static pressure (in hectopascal) and temperature (in degrees Celsius), respectively, to specified ROS topics, while also adding some noise to the readings.

3. **DiffPressureSensor (Differential Pressure Sensor)**: The `DiffPressureSensor` is a sensor class that periodically publishes the differential pressure (in hectopascal) of the UAV to a specified ROS topic, incorporating some noise into the measurements.

4. **IceStatusSensor**: The `IceStatusSensor` is a sensor class that monitors and publishes the status and RPM (Revolutions Per Minute) of the Internal Combustion Engine (ICE) of a UAV, also simulating different operating modes such as stall and normal mode, to a specified ROS topic.

5. **ImuSensor (Inertial Measurement Unit)**: The `ImuSensor` is a sensor class that publishes inertial measurement unit (IMU) data, including linear acceleration and angular velocity in three dimensions, at a specified frequency to a given ROS topic.

6. **VelocitySensor**: The `VelocitySensor` is a sensor class in ROS that simulates and publishes linear and angular velocity data in three dimensions at a specified frequency on a specific ROS topic.

7. **GpsSensor (Global Positioning System Sensor)**: The `GpsSensor` is a sensor class in ROS that simulates and publishes geographic position data including latitude, longitude, and altitude at a specified frequency on a specific ROS topic.

8. **MagSensor (Magnetometer Sensor)**: The `MagSensor` class is a ROS sensor simulation that periodically publishes the magnetic field data, considering the current geographical position and attitude, on a specified topic, also introducing some level of noise.

9. **EscStatusSensor (Electronic Speed Controller Status Sensor)**: The `EscStatusSensor` class is a ROS sensor simulation that periodically publishes Electronic Speed Controller (ESC) status data, including RPM, temperature, voltage, and current for each ESC, on a specified topic.

10. **FuelTankSensor**: The `FuelTankSensor` class simulates a ROS sensor that periodically publishes the level of fuel in the tank as a percentage on a specified topic.

11. **BatteryInfoSensor**: The `BatteryInfoSensor` class emulates a ROS sensor that periodically publishes the current battery state, including the voltage and capacity, on a specified topic.

These sensors, in combination, provide a comprehensive set of data that can be used to simulate and control the flight of a UAV or VTOL aircraft.


## Sensors Class

The `Sensors` class aggregates all sensor simulation modules used in the UAV and VTOL aircraft simulation. It acts as a centralized management system for initiating, enabling and publishing sensor data.

The class is initialized with a ROS node handle pointer, which is used to bind each sensor to its respective ROS topic, with a specific publication frequency.

Each sensor is initialized as an attribute of the `Sensors` class, allowing easy access and management of all sensor data within the class.

### Class Methods

- **Constructor**: Takes a `ros::NodeHandle` pointer, assigns this to each sensor object, along with the appropriate ROS topic and publication frequency.

- **init()**: Takes a `std::shared_ptr` to a `UavDynamicsSimBase` object, used for fetching UAV state data. It also retrieves parameters (latitude, longitude, and altitude references, along with sensor enable flags) from the ROS parameter server and initializes the `geodeticConverter` object. If a sensor is enabled, its `enable()` method is called.

- **publishStateToCommunicator()**: Retrieves data from the simulator, converts it to the correct coordinate system (either PX4 or ROS), calculates temperature, absolute pressure and differential pressure, and publishes this data to the relevant ROS topics via each sensor's `publish()` method.


### Class Attributes

- **Sensor objects**: `Sensors` has a number of sensor objects as its attributes, including `AttitudeSensor`, `PressureSensor`, `TemperatureSensor`, `DiffPressureSensor`, `IceStatusSensor`, `ImuSensor`, `VelocitySensor_`, `GpsSensor`, `MagSensor`, `EscStatusSensor`, `FuelTankSensor`, `BatteryInfoSensor`. These represent the various sensors that are part of the UAV or VTOL aircraft simulation.

- **UavDynamicsSimBase pointer**: `_uavDynamicsSim` is a shared pointer to a `UavDynamicsSimBase` object. This object represents the UAV's dynamics simulator, providing access to the vehicle's current state.
