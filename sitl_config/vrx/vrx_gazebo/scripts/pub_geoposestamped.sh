#!/bin/bash
# Utility script for publishing landmark identification and localization

rostopic pub -r 1 /vrx/perception/landmark geographic_msgs/GeoPoseStamped '{header: {stamp: now, frame_id: "red_mark"}, pose: {position: {latitude: 21.30996, longitude: -157.8901, altitude: 0.0}}}'

#rostopic pub -r 1 /vrx/perception/landmark geographic_msgs/GeoPoseStamped '{header: {stamp: now, frame_id: "black_totem"}, pose: {position: {latitude: 21.30996, longitude: -157.8901, altitude: 0.0}}}'
