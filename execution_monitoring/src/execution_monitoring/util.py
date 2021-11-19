#!/usr/bin/env python

def parse_mission_name(mission_name):
    return mission_name.data.strip().replace(" ", "_").replace(",", "_").replace("__", "_").replace(":", "_")
