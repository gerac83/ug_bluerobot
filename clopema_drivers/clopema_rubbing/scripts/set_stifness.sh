#!/usr/bin/env bash

rosservice call --wait "/r${1}_gripper/SetRubbingStiffness" "stiffness: $2"

