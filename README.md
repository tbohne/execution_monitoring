# Execution Monitoring Framework

Fully integrated plan execution, monitoring and resolution framework capable of overcoming some of the typical limitations of long-term autonomous mobile outdoor robots. Detailed background information is available in my [**master's thesis**](https://github.com/tbohne/msc), in the context of which this framework was developed.

## General Dependencies
- [mongodb_store](https://github.com/strands-project/mongodb_store): MongoDB tools for storing and analysing runs of ROS systems
    - branch: `melodic-devel`
- [plan_generation](https://github.com/tbohne/plan_generation):  ROS node that generates / provides action plans based on CSV data
    - branch: `main`

## Dependencies for the Prototype Scenario in the Simulation

- [arox_docker](https://git.ni.dfki.de/arox/arox_docker): dockerization of the AROX (Autonomous Robotic Experimentation Platform)
    - branch: `noetic`
    - compatible branches within docker container:
        - [arox_navigation_flex](https://git.ni.dfki.de/arox/arox_core/arox_navigation_flex): `feature_msc_setup_tim`
        - [arox_launch](https://git.ni.dfki.de/arox/arox_core/arox_launch): `feature_msc_setup_tim`
        - [arox_indoor_navi](https://git.ni.dfki.de/arox/arox_core/arox_indoor_navi): `feature_less_self_scan`
        - [arox_engine](https://git.ni.dfki.de/arox/arox_core/arox_engine): `feature_arox_battery`
        - [arox_performance_parameters ](https://git.ni.dfki.de/arox/arox_core/arox_performance_parameters): `feature_msc_setup_tim`
        - [map_langsenkamp](https://git.ni.dfki.de/zla/map_langsenkamp): `feature_lta_map`
        - [arox_docking](https://git.ni.dfki.de/arox/arox_core/arox_docking): `feature/python2_compatible`
- [arox_description](https://git.ni.dfki.de/arox/arox_core/arox_description): ROS launch files and URDF model for the AROX system
    - branch: `feature_lta_spawn`
- [container_description](https://git.ni.dfki.de/arox/container_description): ROS launch files and URDF model for the mobile container (charge station)
    - branch: `feature_simple_collisions`
- [innok_heros_description](https://git.ni.dfki.de/arox/innok_heros/innok_heros_description): URDF description for Innok Heros robot
    - branch: `arox_noetic`
- [innok_heros_driver](https://git.ni.dfki.de/arox/innok_heros/innok_heros_driver): ROS driver for the Innok Heros robot platform
    - branch: `master`
- [velodyne_simulator](https://bitbucket.org/DataspeedInc/velodyne_simulator/src/master/): URDF and gazebo plugin to provide simulated data from Velodyne laser scanners
    - branch: `master`
- [gazebo_langsenkamp](https://git.ni.dfki.de/zla/gazebo_langsenkamp): Langsenkamp world (test field)
    - branch: `feature_msc_setup_tim`

## Gazebo Models

[Models](https://cloud.dfki.de/owncloud/index.php/s/TBCzjPfZbzEpMfa) to be placed in `/.gazebo/models/`:
- `charger_ground_patch`, `Corn`, `langsenkamp_simulation`, `Pillar_1`
- additionally: `stop_sign` and `jersey_barrier` from the `/models` directory of this repository

## Usage - Prototype Scenario in the Simulation

- run simulation (with GUI): `roslaunch arox_description launch_arox_sim.launch gui:=true`
- optional: [spawn container: `roslaunch container_description spawn.launch`]
- spawn AROX: `roslaunch arox_description spawn.launch`
- run AROX controllers: `roslaunch arox_description run_controllers.launch`
- run docker container named 'arox_msc': `aroxstartdocker arox_msc` (alias)
    - launch outdoor simulation: `roslaunch arox_launch arox_sim_outdoor.launch`

## Usage - Execution Monitoring Framework (Within Docker Container)

Launch complete framework: 
```
$ roslaunch execution_monitoring execution_monitoring.launch
```
### Components that can be (de)activated in the launch file:
- plan generation
- autonomous (un)docking
- `MongoDB` logging
- dummy scanner
- failure resolution
- failure simulation
- monitoring nodes (individually)
- energy consumption model
- LTA experiments
- `pointcloud-to-laserscan`

## Exploration GUI

`http://localhost/exploration_gui/`

## Settings for AROX Battery (Non-Defaults)

- `discharge_rate`: 0.1
- `battery_charging_level`: 100

Configurable via:
```
$ rosrun rqt_reconfigure rqt_reconfigure
```

## Useful Topics to Monitor

- state of mission: `rostopic echo /arox/ongoing_operation`
- battery charge level: `rostopic echo /arox/battery_param`

## Control AROX

Launch keyboard control:
```
$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

## Visualize Sensor Data

- [rViz](https://wiki.ros.org/rviz)
    - fixed frame: `map`
    - add sensors by topic, e.g., `/velodyne_points` to visualize the point cloud recorded by the Velodyne 3D lidar sensor
    - or open the provided config `msc_conf.rviz`

## Open Container (Lower Ramp)

```
$ rostopic pub -1 /container/rampA_position_controller/command std_msgs/Float64 "data: 1.57"
```

## Clear Costmaps (`move_base_flex`)

```
$ rosservice call /move_base_flex/clear_costmaps "{}"
```

## Robot-Human Communication Module

```
$ rosrun execution_monitoring operator_communication.py
```

## Send 'End-of-Episode' Signal

```
$ rostopic pub -1 /end_of_episode std_msgs/String "EOE"
```

## Monitoring States

Manually trigger *CONTINGENCY* or *CATASTROPHE*:
```
$ rostopic pub -1 "/contingency_preemption" std_msgs/String contingency
$ rostopic pub -1 "/catastrophe_preemption" std_msgs/String catastrophe
```

## Visualize State of Hierarchical SMACH (Introspection Server)

```
$ rosrun smach_viewer smach_viewer.py
```

## Architecture

- *high level (plan execution + monitoring):* ![](img/SMACH_high_level.png)
- *low level (operational model):* ![](img/SMACH_low_level.png)

## Experiments - Simulation of LTA Challenges

### Sensor Failures

- **total sensor failure:** `rostopic pub -1 /toggle_simulated_total_sensor_failure std_msgs/String fail`
- **empty list of range values:** `rostopic pub -1 /toggle_simulated_empty_ranges std_msgs/String fail`
- **predominantly impermissible values:** `rostopic pub -1 /toggle_simulated_impermissible_ranges std_msgs/String fail`
- **repeated scan:** `rostopic pub -1 /toggle_simulated_scan_repetition std_msgs/String fail`

### Connection Failures

#### WiFi Failures

- **poor link quality:** `rostopic pub -1 /toggle_simulated_bad_wifi_link std_msgs/String fail`
- **poor signal:** `rostopic pub -1 /toggle_simulated_bad_wifi_signal std_msgs/String fail`
- **poor bit rate:** `rostopic pub -1 /toggle_simulated_bad_wifi_bit_rate std_msgs/String fail`
- **disconnect:** `rostopic pub -1 /toggle_simulated_wifi_disconnect std_msgs/String fail`

#### Internet Failures

- **disconnect:** `rostopic pub -1 /sim_internet_connection_failure std_msgs/String fail`
- **bad download:** `rostopic pub -1 /toggle_simulated_bad_download std_msgs/String fail`
- **bad upload:** `rostopic pub -1 /toggle_simulated_bad_upload std_msgs/String fail`

#### GNSS Failures / Options

- **disconnect / timeout:** `rostopic pub -1 /toggle_simulated_timeout_failure std_msgs/String fail`
- **good quality** `rostopic pub -1 /set_simulated_good_quality std_msgs/String fail`
- **medium quality:** `rostopic pub -1 /set_simulated_med_quality std_msgs/String fail`
- **low quality:** `rostopic pub -1 /set_simulated_low_quality std_msgs/String fail`
- **unknown status:** `rostopic pub -1 /set_simulated_unknown_status std_msgs/String fail`
- **no fix:** `rostopic pub -1 /set_simulated_no_fix std_msgs/String fail`
- **no RTK:** `rostopic pub -1 /set_simulated_no_rtk std_msgs/String fail`
- **unknown service:** `rostopic pub -1 /toggle_simulated_unknown_service std_msgs/String fail`
- **infeasible lat / lng:** `rostopic pub -1 /toggle_simulated_infeasible_lat_lng std_msgs/String fail`
- **variance history failure:** `rostopic pub -1 /toggle_simulated_variance_history_failure std_msgs/String fail`
- **high deviation:** `rostopic pub -1 /toggle_simulated_high_deviation std_msgs/String fail`

### Drastic Weather Changes

- **heavy rain:** `rostopic pub -1 /toggle_rain_sim std_msgs/String fail`
- **heavy snow:** `rostopic pub -1 /toggle_snow_sim std_msgs/String fail`
- **gale:** `rostopic pub -1 /toggle_wind_sim std_msgs/String fail`
- **low temp.:** `rostopic pub -1 /toggle_low_temp_sim std_msgs/String fail`
- **thunderstorm:** `rostopic pub -1 /toggle_thuderstorm_sim std_msgs/String fail`
- **sunset:** `rostopic pub -1 /toggle_sunset_sim std_msgs/String fail`

### Data Management Failures

- **full memory:** prepare full USB flash drive
    - find out device: `fdisk -l`, e.g. `/dev/sdd1`
        - if necessary: `lsblk -f`
        - `umount /dev/sdd1 MOUNTPOINT`
    - create directory for flash drive: `mkdir /mnt/usb`
    - mount flash drive: `mount /dev/sdd1 /mnt/usb`
    - set `MONITOR_DRIVE` to `/mnt/usb`
    - then `rostopic pub -1 /sim_full_disk_failure std_msgs/String fail`
- **scan logging failure:** `rostopic pub -1 /toggle_simulated_scan_logging_failure std_msgs/String fail`

### Localization Failures

- **odometry-GNSS distance divergence (type 1):** `rostopic pub -1 /wheel_movement_without_pos_change std_msgs/String fail`
- **odometry-GNSS distance divergence (type 2):**  `rostopic pub -1 /pos_change_without_wheel_movement std_msgs/String fail`
- **interpolated GNSS and IMU / odometry yaw divergence:** `rostopic pub -1 /yaw_divergence std_msgs/String fail`
- **IMU acceleration although no active nav goal:** `rostopic pub -1 /moving_although_standing_still_imu std_msgs/String fail`
    - *note: only working if both the active and passive history is complete (e.g. 1500)*
- **odometry twist although no active nav goal:** `rostopic pub -1 /moving_although_standing_still_odom std_msgs/String fail`

### Plan Deployment Failures

- **extended idle time:** `rostopic pub -1 /sim_extended_idle_time std_msgs/String fail`
- **unavailable plan service:** `rostopic pub -1 /toggle_unavailable_plan_service std_msgs/String fail`
- **empty plan:** `rostopic pub -1 /sim_empty_plan std_msgs/String fail`
- **infeasible plan:** `rostopic pub -1 /sim_infeasible_plan std_msgs/String fail`

### Navigation Failures

- **static obstacles:** `rostopic pub -1 /spawn_static_obstacles std_msgs/String scene_[one, two, three, four]`
- **robot prison:** `rostopic pub -1 /spawn_robot_prison std_msgs/String fail`
- **navigation failure:** `rostopic pub -1 /trigger_nav_fail std_msgs/String fail`

### Charging Failures

- **undocking failure (raised ramp):** `rostopic pub -1 /sim_undocking_failure std_msgs/String fail`
- **docking failure (raised ramp):** `rostopic pub -1 /sim_docking_failure_raised_ramp std_msgs/String fail`
- **docking failure (wrong base pose):** `rostopic pub -1 /sim_docking_failure_base_pose std_msgs/String fail`
- **charging failure:** `rostopic pub -1 /sim_charging_failure std_msgs/String fail`

### Power Management Failures

- **contingency:** `rostopic pub -1 /sim_power_management_contingency std_msgs/String fail`
- **catastrophe:** `rostopic pub -1 /sim_power_management_catastrophe std_msgs/String fail`
