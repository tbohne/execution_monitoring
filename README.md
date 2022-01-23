# Execution Monitoring Node

**High-level state machine providing plan execution and monitoring for LTA plant observation with a mobile robot.**

## Dependencies

- [plan_generation](https://github.com/tbohne/plan_generation): `main`
- [arox_navigation_flex](https://git.ni.dfki.de/arox/arox_core/arox_navigation_flex): `feature_msc_setup_tim`
- [arox_performance_parameters ](https://git.ni.dfki.de/arox/arox_core/arox_performance_parameters): `feature_msc_setup_tim`

## Usage

- **manual:**
    - run high-level execution monitoring state machine: `rosrun execution_monitoring high_level_smach.py`
    - run action server providing the dummy scanner: `rosrun execution_monitoring dummy_scanner.py`
    - run fake RIEGL publisher republishes Velodyne scans): `rosrun execution_monitoring republish_velodyne.py`
    - run monitoring node: `rosrun execution_monitoring monitoring.py`
    - run resolver node: `rosrun execution_monitoring resolver.py`
    - run battery model (energy consumption): `rosrun arox_engine arox_battery.py`
        - configurable via `rosrun rqt_reconfigure rqt_reconfigure`
- **with launch file (including plan generation)**:
    - `rosrun execution_monitoring gps_simulator.py`
    - `roslaunch execution_monitoring execution_monitoring.launch`
- OS-specific WiFi monitoring node (Ubuntu version):
    - `rosrun execution_monitoring wifi_monitor.py `

## Simulate LTA problems

- **sensor failures**
    - **total sensor failure:** `rostopic pub -1 /toggle_simulated_total_sensor_failure std_msgs/String fail`
    - **empty list of range values:** `rostopic pub -1 /toggle_simulated_empty_ranges std_msgs/String fail`
    - **predominantly impermissible values:** `rostopic pub -1 /toggle_simulated_impermissible_ranges std_msgs/String fail`
    - **repeated scan:** `rostopic pub -1 /toggle_simulated_scan_repetition std_msgs/String fail`
- **connection failures**
    - **WiFi failures**
        - **poor link quality:** `rostopic pub -1 /toggle_simulated_bad_wifi_link std_msgs/String fail`
        - **poor signal:** `rostopic pub -1 /toggle_simulated_bad_wifi_signal std_msgs/String fail`
        - **poor bit rate:** `rostopic pub -1 /toggle_simulated_bad_wifi_bit_rate std_msgs/String fail`
        - **disconnect:** `rostopic pub -1 /toggle_simulated_wifi_disconnect std_msgs/String fail`
    - **internet failures**
        - **disconnect:** just cut the connection
        - **bad download:** `rostopic pub -1 /toggle_simulated_bad_download std_msgs/String fail`
        - **bad upload:** `rostopic pub -1 /toggle_simulated_bad_upload std_msgs/String fail`
    - **GNSS failures / options**
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
- **drastic weather changes**
    - **heavy rain:** `rostopic pub -1 /toggle_rain_sim std_msgs/String fail`
    - **heavy snow:** `rostopic pub -1 /toggle_snow_sim std_msgs/String fail`
    - **gale:** `rostopic pub -1 /toggle_wind_sim std_msgs/String fail`
    - **low temp.:** `rostopic pub -1 /toggle_low_temp_sim std_msgs/String fail`
    - **thunderstorm:** `rostopic pub -1 /toggle_thuderstorm_sim std_msgs/String fail`
    - **sunset:** `rostopic pub -1 /toggle_sunset_sim std_msgs/String fail`
- **data management failures**
    - **full memory:** prepare full USB flash drive and configure `MONITOR_DRIVE` (path to monitor) accordingly
        - find out device: `fdisk -l`, e.g. `/dev/sdd1`
        - create directory for flash drive: `mkdir /mnt/usb`
        - mount flash drive: `mount /dev/sdd1 /mnt/usb`
        - set `MONITOR_DRIVE` to `/mnt/usb`
    - **scan logging failure:** `rostopic pub -1 /toggle_simulated_scan_logging_failure std_msgs/String fail`
- **localization failures**
    - **odometry-GNSS distance divergence (type 1):** `rostopic pub -1 /wheel_movement_without_pos_change std_msgs/String fail`
    - **odometry-GNSS distance divergence (type 2):**  `rostopic pub -1 /pos_change_without_wheel_movement std_msgs/String fail`
    - **interpolated GNSS and IMU / odometry yaw divergence:** `rostopic pub -1 /yaw_divergence std_msgs/String fail`
    - **IMU acceleration although no active nav goal:** `rostopic pub -1 /moving_although_standing_still_imu std_msgs/String fail`
        - *note: only working if both the active and passive history is complete (e.g. 1500)*
    - **odometry twist although no active nav goal:** `rostopic pub -1 /moving_although_standing_still_odom std_msgs/String fail`
- **plan deployment failures**
    - **extended idle time:** `rostopic pub -1 /sim_extended_idle_time std_msgs/String fail`
    - **unavailable plan service:** `rostopic pub -1 /toggle_unavailable_plan_service std_msgs/String fail`
    - **empty plan:** `rostopic pub -1 /sim_empty_plan std_msgs/String fail`
    - **infeasible plan:** `rostopic pub -1 /sim_infeasible_plan std_msgs/String fail`

## Communication with Human Operator

- robot-human communication module: `rosrun execution_monitoring operator_communication.py`
- human operator indicates that the problem is solved: `rostopic pub -1 /problem_solved std_msgs/String solved`

## Monitoring States

- manually trigger *CONTINGENCY* or *CATASTROPHE*:
    - **contingency detected**: `rostopic pub -1 "/contingency_preemption" std_msgs/String contingency`
    - **catastrophe detected**: `rostopic pub -1 "/catastrophe_preemption" std_msgs/String catastrophe`

## Visualize State of Hierarchical SMACH

`rosrun smach_viewer smach_viewer.py`

## Architecture

- high level: ![](img/SMACH_high_level.jpg)
- low level: ![](img/SMACH_low_level.jpg)
