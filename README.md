# Execution Monitoring Node

**High-level state machine providing plan execution and monitoring for LTA plant observation with a mobile robot.**

## Dependencies

- [plan_generation](https://github.com/tbohne/plan_generation): `main`
- [arox_navigation_flex](https://git.ni.dfki.de/arox/arox_core/arox_navigation_flex): `feature_msc_setup_tim`
- [arox_performance_parameters ](https://git.ni.dfki.de/arox/arox_core/arox_performance_parameters): `feature_performace_parameters`

## Usage

`rosrun execution_monitoring execution_monitoring.py`

## Architecture

![](SMACH.jpg)