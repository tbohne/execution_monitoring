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
    - `roslaunch execution_monitoring execution_monitoring.launch`

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
- **data management failures**
    - **full memory:** prepare full USB stick and configure `MONITOR_DRIVE` (path to monitor) accordinglly
        - find out device: `fdisk -l`, e.g. `/dev/sdd1`
        - create directory for USB stick: `mkdir /mnt/usb`
        - mount USB stick: `mount /dev/sdd1 /mnt/usb`
        - set `MONITOR_DRIVE` to `/mnt/usb`
    - **scan logging failure:** `rostopic pub -1 /toggle_simulated_scan_logging_failure std_msgs/String fail`

## Communication with Human Operator

- robot requests help: `rostopic echo "/request_help"`
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
