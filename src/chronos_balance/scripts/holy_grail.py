#!/usr/bin/env python3
"""
ik_step_physical.py — 10-DOF Humanoid Step Controller for Real Hardware (AX-12A)
"""

import math
import time
import threading
import sys
import os

try:
    import msvcrt
    is_windows = True
except ImportError:
    is_windows = False
    import termios
    import tty
    import select
from dynamixel_sdk import *

# =============================================================================
# 1. HARDWARE CONFIGURATION & TRANSLATION
# =============================================================================
TICKS_PER_RADIAN = 1024 / (300.0 * math.pi / 180.0)


ADDR_AX_GOAL_POSITION = 30
LEN_AX_GOAL_POSITION  = 2
PROTOCOL_VERSION      = 1.0
BAUDRATE              = 1000000  
DEVICENAME            = '/dev/ttyUSB0' # Detected Linux serial port

ROBOT_CONFIG = {
    # --- UPPER BODY (ARMS) ---
    1:  {"name": "Shoulder Pitch R", "offset": 818, "dir": 1},
    2:  {"name": "Shoulder Pitch L", "offset": 506, "dir": -1},
    3:  {"name": "Shoulder Roll R",  "offset": 349, "dir": 1},
    4:  {"name": "Shoulder Roll L",  "offset": 349, "dir": -1},
    5:  {"name": "Elbow R",          "offset": 28,  "dir": 1},
    6:  {"name": "Elbow L",          "offset": 682, "dir": 1},

    # --- YAW / WAIST ---
    7:  {"name": "Arm/Torso L",      "offset": 671, "dir": 1},
    8:  {"name": "Arm/Torso R",      "offset": 657, "dir": 1},
    
    # --- RIGHT LEG (Odds) ---
    9:  {"name": "Hip Roll R",       "offset": 497, "dir": 1},
    11: {"name": "Hip Pitch R",      "offset": 525, "dir": -1},
    13: {"name": "Knee R",           "offset": 513, "dir": -1},
    15: {"name": "Ankle Pitch R",    "offset": 520, "dir": 1},
    17: {"name": "Ankle Roll R",     "offset": 488, "dir": 1},
    
    # --- LEFT LEG (Evens) ---
    10: {"name": "Hip Roll L",       "offset": 402, "dir": 1},
    12: {"name": "Hip Pitch L",      "offset": 498, "dir": -1},
    14: {"name": "Knee L",           "offset": 509, "dir": 1},
    16: {"name": "Ankle Pitch L",    "offset": 194, "dir": -1},
    18: {"name": "Ankle Roll L",     "offset": 529, "dir": 1}
}

SHUTDOWN_TICKS = {
    1: 863, 2: 525, 3: 352, 4: 350, 5: 30, 6: 685,
    7: 662, 8: 649,
    9: 508, 10: 405,
    11: 264, 12: 752,
    13: 35, 14: 987,
    15: 750, 16: 0,
    17: 506, 18: 522
}

DAB_TICKS = {
    1: 1015, 2: 59, 3: 426, 4: 588, 5: 37, 6: 703,
    7: 671, 8: 675,
    9: 496, 10: 403,
    11: 403, 12: 607,
    13: 514, 14: 507,
    15: 498, 16: 531,
    17: 488, 18: 525
}

HANDSHAKE_TICKS = {
    1: 1000, 2: 525, 3: 340, 4: 390, 5: 28, 6: 647,
    7: 684, 8: 658,
    9: 498, 10: 402,
    11: 525, 12: 493,
    13: 513, 14: 505,
    15: 508, 16: 509,
    17: 493, 18: 527
}

PUSHUP_START_TICKS = {
    1: 1001, 2: 339, 3: 232, 4: 497, 5: 184, 6: 523,
    7: 684, 8: 658,
    9: 498, 10: 402,
    11: 525, 12: 493,
    13: 513, 14: 505,
    15: 746, 16: 302,
    17: 491, 18: 523
}

# Note: Push-up bottom position ticks are now computed dynamically inside push_up() 
# using the Law of Cosines (Cosine Rule) and Tangent Rule for precise kinematics!

is_shutting_down = False
is_paused = False
g_portHandler = None
g_packetHandler = None
g_groupSyncWrite = None
target_yaw_left = 0.0
target_yaw_right = 0.0

def rad_to_tick(motor_id, radian_cmd):
    """Translates IK radians to AX-12A ticks."""
    config = ROBOT_CONFIG[motor_id]
    tick_delta = radian_cmd * TICKS_PER_RADIAN * config["dir"]
    target_tick = int(config["offset"] + tick_delta)
    return max(0, min(1023, target_tick))

# =============================================================================
# 2. LEG LENGTHS & TARGETS
# =============================================================================
L1 = 0.11
L2 = 0.116

# ==========================================================
# Gait Parameters
# ==========================================================
RIGHT_STEP_LENGTH = 0.10
LEFT_STEP_LENGTH = 0.10
SUPPORT_RATIO = 0.75
STAGE6_SUPPORT_OFFSET = 0.02  

KNEE_MARGIN = 0.006 
STAGE6_KNEE_MARGIN = 0.012    

MAX_H_DIFF = 0.05      

MAX_TURN_STEP_ANGLE = 0.3
DEFAULT_TURN_LEFT_ANGLE = 0.3
DEFAULT_TURN_RIGHT_ANGLE = 0.3
TURN_LIFT_HEIGHT = 0.15


target_h_left = 0.23
target_h_right = 0.23
target_shift_left = 0.0
target_shift_right = 0.0
target_x_left = 0.0

target_x_right = 0.0
target_arm_pitch_left = 0.0
target_arm_pitch_right = 0.0

# =============================================================================
# 3. SYSTEMATIC 6-STAGE GAIT STATE MACHINE
# =============================================================================
class AbortWalk(Exception): pass
abort_walk = False

g_crouch_offset = 0.0
g_arm_pitch_offset = 0.0

def set_targets(h_l, h_r, shift_l, shift_r, x_l, x_r, delay):
    global target_h_left, target_h_right, target_shift_left, target_shift_right, target_x_left, target_x_right
    if abort_walk: raise AbortWalk()
    
    target_h_left = max(0.14, h_l - g_crouch_offset)
    target_h_right = max(0.14, h_r - g_crouch_offset)
    target_shift_left = shift_l
    target_shift_right = shift_r
    target_x_left = x_l
    target_x_right = x_r
    time.sleep(delay)

def transition_yaw(yaw_l, yaw_r, duration):
    """Smoothly interpolates yaw targets over a precise duration to eliminate jerky snaps."""
    global target_yaw_left, target_yaw_right
    if abort_walk: raise AbortWalk()
    
    start_yaw_l = target_yaw_left
    start_yaw_r = target_yaw_right
    
    steps = int(duration * 20)  # 20 steps per second
    if steps <= 0: steps = 1
    dt = duration / steps
    
    for i in range(1, steps + 1):
        if abort_walk: raise AbortWalk()
        t = i / float(steps)
        # Cosine ease-in/out for extra smoothness
        ease = (1 - math.cos(t * math.pi)) / 2.0
        
        target_yaw_left = start_yaw_l + ease * (yaw_l - start_yaw_l)
        target_yaw_right = start_yaw_r + ease * (yaw_r - start_yaw_r)
        
        time.sleep(dt)

def transition_to(h_l, h_r, shift_l, shift_r, x_l, x_r, duration):
    """Smoothly interpolates targets over a precise duration to eliminate jerky snaps."""
    global target_h_left, target_h_right, target_shift_left, target_shift_right, target_x_left, target_x_right
    
    start_h_l = target_h_left
    start_h_r = target_h_right
    start_shift_l = target_shift_left
    start_shift_r = target_shift_right
    start_x_l = target_x_left
    start_x_r = target_x_right
    
    steps = int(duration * 20)  # 20 steps per second
    if steps <= 0: steps = 1
    dt = duration / steps
    
    for i in range(1, steps + 1):
        t = i / float(steps)
        # Cosine ease-in/out for extra smoothness!
        ease = (1 - math.cos(t * math.pi)) / 2.0
        
        cur_h_l = start_h_l + ease * (h_l - start_h_l)
        cur_h_r = start_h_r + ease * (h_r - start_h_r)
        cur_shift_l = start_shift_l + ease * (shift_l - start_shift_l)
        cur_shift_r = start_shift_r + ease * (shift_r - start_shift_r)
        cur_x_l = start_x_l + ease * (x_l - start_x_l)
        cur_x_r = start_x_r + ease * (x_r - start_x_r)
        
        set_targets(cur_h_l, cur_h_r, cur_shift_l, cur_shift_r, cur_x_l, cur_x_r, dt)

def generate_stage5(step_length):
    x_r = SUPPORT_RATIO * step_length
    x_l = -(step_length - x_r)
    hmax_l = math.sqrt((L1 + L2)**2 - x_l*x_l)
    hmax_r = math.sqrt((L1 + L2)**2 - x_r*x_r)
    h_l = hmax_l - KNEE_MARGIN
    h_r = hmax_r - KNEE_MARGIN
    if abs(h_l-h_r) > MAX_H_DIFF:
        if h_l > h_r: h_l = h_r + MAX_H_DIFF
        else: h_r = h_l + MAX_H_DIFF
    shift_l = 0.10
    shift_r = 0.10
    return h_l,h_r,shift_l,shift_r,x_l,x_r

def generate_stage6(step_length):
    x_r = STAGE6_SUPPORT_OFFSET
    x_l = -(step_length - x_r)
    hmax_l = math.sqrt((L1+L2)**2 - x_l*x_l)
    hmax_r = math.sqrt((L1+L2)**2 - x_r*x_r)
    h_l = hmax_l - STAGE6_KNEE_MARGIN
    h_r = hmax_r - STAGE6_KNEE_MARGIN
    if abs(h_l-h_r) > MAX_H_DIFF:
        if h_l > h_r: h_l = h_r + MAX_H_DIFF
        else: h_r = h_l + MAX_H_DIFF
    shift_l = -0.30
    shift_r = -0.30
    return h_l,h_r,shift_l,shift_r,x_l,x_r

def generate_stage7(step_length):
    x_l = SUPPORT_RATIO * step_length
    x_r = -(step_length - x_l)
    hmax_l = math.sqrt((L1+L2)**2 - x_l*x_l)
    hmax_r = math.sqrt((L1+L2)**2 - x_r*x_r)
    h_l = hmax_l - KNEE_MARGIN
    h_r = hmax_r - KNEE_MARGIN
    if abs(h_l-h_r) > MAX_H_DIFF:
        if h_l > h_r: h_l = h_r + MAX_H_DIFF
        else: h_r = h_l + MAX_H_DIFF
    shift_l_start = -0.40
    shift_l_end   = -0.35
    shift_r = -0.30
    return (h_l, h_r, shift_l_start, shift_l_end, shift_r, x_l, x_r)

def generate_stage8(step_length):
    x_l = SUPPORT_RATIO * step_length
    x_r = -(step_length - x_l)
    hmax_l = math.sqrt((L1+L2)**2 - x_l*x_l)
    hmax_r = math.sqrt((L1+L2)**2 - x_r*x_r)
    h_l = hmax_l - KNEE_MARGIN
    h_r = hmax_r - KNEE_MARGIN
    if abs(h_l-h_r) > MAX_H_DIFF:
        if h_l > h_r: h_l = h_r + MAX_H_DIFF
        else: h_r = h_l + MAX_H_DIFF
    shift_l = -0.10
    shift_r = -0.10
    return h_l,h_r,shift_l,shift_r,x_l,x_r

def generate_stage9(step_length):
    x_l = STAGE6_SUPPORT_OFFSET
    x_r = -(step_length - x_l)
    hmax_l = math.sqrt((L1+L2)**2 - x_l*x_l)
    hmax_r = math.sqrt((L1+L2)**2 - x_r*x_r)
    h_l = hmax_l - STAGE6_KNEE_MARGIN
    h_r = hmax_r - STAGE6_KNEE_MARGIN
    if abs(h_l-h_r) > MAX_H_DIFF:
        if h_l > h_r: h_l = h_r + MAX_H_DIFF
        else: h_r = h_l + MAX_H_DIFF
    shift_l = 0.30
    shift_r = 0.30
    return h_l,h_r,shift_l,shift_r,x_l,x_r

def generate_stage10(step_length):
    x_r = SUPPORT_RATIO * step_length
    x_l = -(step_length - x_r)
    hmax_l = math.sqrt((L1+L2)**2 - x_l*x_l)
    hmax_r = math.sqrt((L1+L2)**2 - x_r*x_r)
    h_l = hmax_l - KNEE_MARGIN
    h_r = hmax_r - KNEE_MARGIN
    if abs(h_l-h_r) > MAX_H_DIFF:
        if h_l > h_r: h_l = h_r + MAX_H_DIFF
        else: h_r = h_l + MAX_H_DIFF
    shift_r_start = 0.40
    shift_r_end   = 0.35
    shift_l = 0.30
    return (h_l, h_r, shift_l, shift_r_start, shift_r_end, x_l, x_r)

def swing_trajectory_right(duration):
    global target_arm_pitch_left, target_arm_pitch_right
    steps = 50
    dt = duration / steps
    h_l_final, h_r_final, shift_l_final, shift_r_final, x_l_final, x_r_final = generate_stage5(RIGHT_STEP_LENGTH)
        
    for i in range(steps + 1):
        t = i / float(steps)
        cyc_t = t - (math.sin(2 * math.pi * t) / (2 * math.pi))
        cyc_lift = (1 - math.cos(2 * math.pi * t)) / 2.0
        
        x_r = cyc_t * x_r_final
        x_l = cyc_t * x_l_final
        h_linear = 0.15 + cyc_t * (h_r_final - 0.15)
        h_r = h_linear - 0.05 * cyc_lift
        com_lateral_sway = 0.03 * cyc_lift
        shift_l = 0.30
        shift_r = (0.40 - cyc_t * 0.05) + com_lateral_sway
        x_com = 2.0 + 2.0 * t
        com_bump = 0.015625 * ((x_com - 4.0) ** 2) * (x_com - 2.0)

        h_l = h_l_final + 0.5 * com_bump
        
        target_arm_pitch_right = -math.sin(math.pi * t) * 0.4
        target_arm_pitch_left  =  math.sin(math.pi * t) * 0.4
        
        set_targets(h_l, h_r, shift_l, shift_r, x_l, x_r, dt)

def swing_trajectory_left(duration):
    global target_arm_pitch_left, target_arm_pitch_right
    steps = 50
    dt = duration / steps
    h_l_start, h_r_start, _, _, x_l_start, x_r_start = generate_stage6(RIGHT_STEP_LENGTH)
    (h_l_final, h_r_final, shift_l_start, shift_l_end, shift_r_const, x_l_final, x_r_final) = generate_stage7(LEFT_STEP_LENGTH)
    
    for i in range(steps + 1):
        t = i / float(steps)
        cyc_t = t - (math.sin(2 * math.pi * t) / (2 * math.pi))
        cyc_lift = (1 - math.cos(2 * math.pi * t)) / 2.0
        
        x_l = x_l_start + cyc_t * (x_l_final - x_l_start)
        x_r = x_r_start + cyc_t * (x_r_final - x_r_start)
        com_bump = 0.02 * math.sin(math.pi * t)
        com_lateral_sway = 0.03 * cyc_lift
        h_linear = h_l_start + cyc_t * (h_l_final - h_l_start)
        h_l = h_linear - 0.05 * cyc_lift
        h_r = h_r_final + 0.5 * com_bump

        shift_l = shift_l_start + cyc_t * (shift_l_end - shift_l_start) - com_lateral_sway
        shift_r = shift_r_const
        
        target_arm_pitch_left  = -math.sin(math.pi * t) * 0.4
        target_arm_pitch_right =  math.sin(math.pi * t) * 0.4
        
        set_targets(h_l, h_r, shift_l, shift_r, x_l, x_r, dt)

def swing_trajectory_right_backward(duration):
    global target_arm_pitch_left, target_arm_pitch_right
    steps = 50
    dt = duration / steps
    
    h_l_start = target_h_left
    h_r_start = target_h_right
    x_l_start = target_x_left
    x_r_start = target_x_right
    shift_l_start = target_shift_left
    shift_r_start = target_shift_right
    
    for i in range(steps + 1):
        t = i / float(steps)
        cyc_t = t - (math.sin(2 * math.pi * t) / (2 * math.pi))
        cyc_lift = (1 - math.cos(2 * math.pi * t)) / 2.0
        
        x_r = x_r_start - cyc_t * x_r_start
        x_l = x_l_start - cyc_t * x_l_start
        
        h_linear = h_r_start + cyc_t * (0.22 - h_r_start)
        h_r = h_linear - 0.07 * cyc_lift
        com_lateral_sway = 0.03 * cyc_lift
        
        shift_l = shift_l_start + cyc_t * (0.0 - shift_l_start)
        shift_r = shift_r_start + cyc_t * (0.0 - shift_r_start) + com_lateral_sway
        
        x_com = 2.0 + 2.0 * t
        com_bump = 0.015625 * ((x_com - 4.0) ** 2) * (x_com - 2.0)
        h_l = h_l_start + cyc_t * (0.22 - h_l_start) + 0.5 * com_bump
        
        target_arm_pitch_right = math.sin(math.pi * t) * 0.3
        target_arm_pitch_left = -math.sin(math.pi * t) * 0.3
        
        set_targets(h_l, h_r, shift_l, shift_r, x_l, x_r, dt)

def swing_trajectory_left_backward(duration):
    global target_arm_pitch_left, target_arm_pitch_right
    steps = 50
    dt = duration / steps
    
    h_l_start = target_h_left
    h_r_start = target_h_right
    x_l_start = target_x_left
    x_r_start = target_x_right
    shift_l_start = target_shift_left
    shift_r_start = target_shift_right
    
    for i in range(steps + 1):
        t = i / float(steps)
        cyc_t = t - (math.sin(2 * math.pi * t) / (2 * math.pi))
        cyc_lift = (1 - math.cos(2 * math.pi * t)) / 2.0
        
        x_l = x_l_start - cyc_t * x_l_start
        x_r = x_r_start - cyc_t * x_r_start
        
        h_linear = h_l_start + cyc_t * (0.22 - h_l_start)
        h_l = h_linear - 0.07 * cyc_lift
        com_lateral_sway = 0.03 * cyc_lift
        
        shift_r = shift_r_start + cyc_t * (0.0 - shift_r_start)
        shift_l = shift_l_start + cyc_t * (0.0 - shift_l_start) - com_lateral_sway
        
        x_com = 2.0 + 2.0 * t
        com_bump = 0.015625 * ((x_com - 4.0) ** 2) * (x_com - 2.0)
        h_r = h_r_start + cyc_t * (0.22 - h_r_start) + 0.5 * com_bump
        
        target_arm_pitch_left = math.sin(math.pi * t) * 0.3
        target_arm_pitch_right = -math.sin(math.pi * t) * 0.3
        
        set_targets(h_l, h_r, shift_l, shift_r, x_l, x_r, dt)

def swing_trajectory_right_continuous(duration):
    global target_arm_pitch_left, target_arm_pitch_right
    steps = 50
    dt = duration / steps
    h_l_start, h_r_start, _, _, x_l_start, x_r_start = generate_stage9(LEFT_STEP_LENGTH)
    (h_l_final, h_r_final, shift_l_const, shift_r_start, shift_r_end, x_l_final, x_r_final) = generate_stage10(RIGHT_STEP_LENGTH)
    
    for i in range(steps + 1):
        t = i / float(steps)
        cyc_t = t - (math.sin(2 * math.pi * t) / (2 * math.pi))
        cyc_lift = (1 - math.cos(2 * math.pi * t)) / 2.0
        
        x_l = x_l_start + cyc_t * (x_l_final - x_l_start)
        x_r = x_r_start + cyc_t * (x_r_final - x_r_start)
        com_bump = 0.02 * math.sin(math.pi * t)
        com_lateral_sway = 0.03 * cyc_lift
        h_linear = h_r_start + cyc_t * (h_r_final - h_r_start)
        h_r = h_linear - 0.05 * cyc_lift
        h_l = h_l_final + 0.5 * com_bump

        shift_r = shift_r_start + cyc_t * (shift_r_end - shift_r_start) + com_lateral_sway
        shift_l = shift_l_const
        
        target_arm_pitch_right = -math.sin(math.pi * t) * 0.4
        target_arm_pitch_left  =  math.sin(math.pi * t) * 0.4
        
        set_targets(h_l, h_r, shift_l, shift_r, x_l, x_r, dt)


def swing_right_forward_align(duration):
    global target_arm_pitch_left, target_arm_pitch_right
    steps = 50
    dt = duration / steps
    
    h_l_start = target_h_left
    h_r_start = target_h_right
    x_l_start = target_x_left
    x_r_start = target_x_right
    shift_l_start = target_shift_left
    shift_r_start = target_shift_right
    
    x_r_final = x_l_start
    
    for i in range(steps + 1):
        t = i / float(steps)
        cyc_t = t - (math.sin(2 * math.pi * t) / (2 * math.pi))
        cyc_lift = (1 - math.cos(2 * math.pi * t)) / 2.0
        
        x_r = x_r_start + cyc_t * (x_r_final - x_r_start)
        x_l = x_l_start
        
        h_linear = h_r_start + cyc_t * (0.22 - h_r_start)
        h_r = h_linear - 0.07 * cyc_lift
        com_lateral_sway = 0.03 * cyc_lift
        
        shift_l = shift_l_start + cyc_t * (0.30 - shift_l_start)
        shift_r = shift_r_start + cyc_t * (0.30 - shift_r_start) + com_lateral_sway
        
        x_com = 2.0 + 2.0 * t
        com_bump = 0.015625 * ((x_com - 4.0) ** 2) * (x_com - 2.0)
        h_l = h_l_start + cyc_t * (0.22 - h_l_start) + 0.5 * com_bump
        
        target_arm_pitch_right = -math.sin(math.pi * t) * 0.3
        target_arm_pitch_left = math.sin(math.pi * t) * 0.3
        
        set_targets(h_l, h_r, shift_l, shift_r, x_l, x_r, dt)

def swing_left_forward_align(duration):
    global target_arm_pitch_left, target_arm_pitch_right
    steps = 50
    dt = duration / steps
    
    h_l_start = target_h_left
    h_r_start = target_h_right
    x_l_start = target_x_left
    x_r_start = target_x_right
    shift_l_start = target_shift_left
    shift_r_start = target_shift_right
    
    x_l_final = x_r_start
    
    for i in range(steps + 1):
        t = i / float(steps)
        cyc_t = t - (math.sin(2 * math.pi * t) / (2 * math.pi))
        cyc_lift = (1 - math.cos(2 * math.pi * t)) / 2.0
        
        x_l = x_l_start + cyc_t * (x_l_final - x_l_start)
        x_r = x_r_start
        
        h_linear = h_l_start + cyc_t * (0.22 - h_l_start)
        h_l = h_linear - 0.07 * cyc_lift
        com_lateral_sway = 0.03 * cyc_lift
        
        shift_r = shift_r_start + cyc_t * (-0.30 - shift_r_start)
        shift_l = shift_l_start + cyc_t * (-0.30 - shift_l_start) - com_lateral_sway
        
        x_com = 2.0 + 2.0 * t
        com_bump = 0.015625 * ((x_com - 4.0) ** 2) * (x_com - 2.0)
        h_r = h_r_start + cyc_t * (0.22 - h_r_start) + 0.5 * com_bump
        
        target_arm_pitch_left = math.sin(math.pi * t) * 0.3
        target_arm_pitch_right = -math.sin(math.pi * t) * 0.3
        
        set_targets(h_l, h_r, shift_l, shift_r, x_l, x_r, dt)

def walk_sequence(num_steps=10):
    print("Resetting to standing pose...")
    transition_to(0.23, 0.23, 0.0, 0.0, 0.0, 0.0, 0.5)
    
    if num_steps < 1:
        return
        
    print("\n=== STEP 1: Initial Right Step ===")
    print("Stage 1: Pre-Crouch (Double Support)...")
    transition_to(0.22, 0.22, 0.0, 0.0, 0.0, 0.0, 0.7)
    
    print("Stage 2: Left Lateral Shift (Weight on Left)...")
    transition_to(0.22, 0.22, 0.30, 0.30, 0.00, 0.00, 0.7) 
    
    print("Stage 3: Lifting Right Leg...")
    transition_to(0.22, 0.12, 0.30, 0.40, 0.00, 0.00, 0.5)

    print("Stage 4: Initial Right Swing...")
    swing_trajectory_right(1.35)
    
    print("Stage 5: Right Touchdown...")
    h_l, h_r, shift_l, shift_r, x_l, x_r = generate_stage5(RIGHT_STEP_LENGTH)
    transition_to(h_l, h_r, shift_l, shift_r, x_l, x_r, 1)

    print("Stage 6: Forward lean (Weight on Right Leg)...")
    h_l, h_r, shift_l, shift_r, x_l, x_r = generate_stage6(RIGHT_STEP_LENGTH)
    transition_to(h_l, h_r, shift_l, shift_r, x_l, x_r, 1)

    print("\n===== END OF STAGE 6 =====")
    print(f"h_l={h_l:.4f}, h_r={h_r:.4f}")
    print(f"shift_l={shift_l:.4f}, shift_r={shift_r:.4f}")
    print(f"x_l={x_l:.4f}, x_r={x_r:.4f}")
    print("==========================\n")

    for i in range(2, num_steps + 1):
        if i % 2 == 0:
            print(f"\n=== STEP {i}: Continuous Left Step ===")
            print("Left Swing...")
            swing_trajectory_left(1.25)
            
            print("Left Touchdown...")
            h_l, h_r, shift_l, shift_r, x_l, x_r = generate_stage8(LEFT_STEP_LENGTH)
            transition_to(h_l, h_r, shift_l, shift_r, x_l, x_r, 0.5)
            
            print("Forward Lean (Weight on Left Leg)...")
            h_l, h_r, shift_l, shift_r, x_l, x_r = generate_stage9(LEFT_STEP_LENGTH)
            transition_to(h_l, h_r, shift_l, shift_r, x_l, x_r, 0.5)
        else:
            print(f"\n=== STEP {i}: Continuous Right Step ===")
            print("Continuous Right Swing...")
            swing_trajectory_right_continuous(1.25)
            
            print("Right Touchdown...")
            h_l, h_r, shift_l, shift_r, x_l, x_r = generate_stage5(RIGHT_STEP_LENGTH)
            transition_to(h_l, h_r, shift_l, shift_r, x_l, x_r, 0.5)
            
            print("Forward Lean (Weight on Right Leg)...")
            h_l, h_r, shift_l, shift_r, x_l, x_r = generate_stage6(RIGHT_STEP_LENGTH)
            transition_to(h_l, h_r, shift_l, shift_r, x_l, x_r, 0.5)
            
    print("\n--- Walk Complete. Aligning Trailing Leg Forward... ---")
    if num_steps % 2 == 1:
        # Right leg is front (-0.01), Left leg is trailing (-0.13)
        print("Committing Weight entirely to Right Leg...")
        transition_to(0.22, 0.22, -0.30, -0.30, target_x_left, target_x_right, 0.5)
        
        print("Swinging Left Leg Forward to Align...")
        swing_left_forward_align(1.35)
    else:
        # Left leg is front (-0.01), Right leg is trailing (-0.13)
        print("Committing Weight entirely to Left Leg...")
        transition_to(0.22, 0.22, 0.30, 0.30, target_x_left, target_x_right, 0.5)
        
        print("Swinging Right Leg Forward to Align...")
        swing_right_forward_align(1.35)
        
    print("Resetting to standing pose...")
    transition_to(0.23, 0.23, 0.0, 0.0, 0.0, 0.0, 1.0)



def dab(duration=1.0, hold_time=2.5):
    global is_paused
    is_paused = True
    time.sleep(0.05) # Allow main loop to pause
    
    print("\n>>> EXECUTING DAB POSE <<<")
    print("Reading physical joint positions for DAB sequence...")
    initial_ticks = {}
    for motor_id in DAB_TICKS.keys():
        tick, result, error = g_packetHandler.read2ByteTxRx(g_portHandler, motor_id, 36)
        if result == COMM_SUCCESS:
            initial_ticks[motor_id] = tick
        else:
            initial_ticks[motor_id] = ROBOT_CONFIG[motor_id]["offset"]
            
    print("Synchronously interpolating to DAB Pose...")
    steps = int(duration * 100)
    for i in range(1, steps + 1):
        start_t = time.perf_counter()
        t = i / float(steps)
        ease = (1 - math.cos(t * math.pi)) / 2.0
        
        g_groupSyncWrite.clearParam()
        for m_id, target_tick in DAB_TICKS.items():
            current_tick = int(initial_ticks[m_id] + ease * (target_tick - initial_ticks[m_id]))
            g_groupSyncWrite.addParam(m_id, [DXL_LOBYTE(current_tick), DXL_HIBYTE(current_tick)])
            
        g_groupSyncWrite.txPacket()
        
        elapsed = time.perf_counter() - start_t
        if elapsed < 0.01:
            time.sleep(0.01 - elapsed)
            
    print(f"--- DAB Pose Reached! Holding for {hold_time}s... ---")
    time.sleep(hold_time)
    
    print("Returning smoothly to Standing Pose...")
    lh_hip, lh_knee, lh_ankle = compute_human_ik(0.23, 0.0)
    rh_hip, rh_knee, rh_ankle = compute_human_ik(0.23, 0.0)
    standing_radians = {
        1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0, 5: 0.0, 6: 0.0,
        7: 0.0, 8: 0.0,
        9: 0.0, 11: rh_hip, 13: -rh_knee, 15: rh_ankle, 17: 0.0,
        10: 0.0, 12: -lh_hip, 14: -lh_knee, 16: lh_ankle, 18: 0.0
    }
    standing_ticks = {m_id: rad_to_tick(m_id, standing_radians[m_id]) for m_id in ROBOT_CONFIG.keys()}
    
    for i in range(1, steps + 1):
        start_t = time.perf_counter()
        t = i / float(steps)
        ease = (1 - math.cos(t * math.pi)) / 2.0
        
        g_groupSyncWrite.clearParam()
        for m_id, start_tick in DAB_TICKS.items():
            target_tick = standing_ticks[m_id]
            current_tick = int(start_tick + ease * (target_tick - start_tick))
            g_groupSyncWrite.addParam(m_id, [DXL_LOBYTE(current_tick), DXL_HIBYTE(current_tick)])
            
        g_groupSyncWrite.txPacket()
        
        elapsed = time.perf_counter() - start_t
        if elapsed < 0.01:
            time.sleep(0.01 - elapsed)
            
    global target_h_left, target_h_right, target_shift_left, target_shift_right, target_x_left, target_x_right
    global target_yaw_left, target_yaw_right, target_arm_pitch_left, target_arm_pitch_right
    target_h_left = 0.23
    target_h_right = 0.23
    target_shift_left = 0.0
    target_shift_right = 0.0
    target_x_left = 0.0
    target_x_right = 0.0
    target_yaw_left = 0.0
    target_yaw_right = 0.0
    target_arm_pitch_left = 0.0
    target_arm_pitch_right = 0.0
    
    is_paused = False
    print("--- Back in Standing Pose. Control resumed. ---\n")

def handshake(duration=1.0, hold_time=1.5):
    global is_paused
    is_paused = True
    time.sleep(0.05) # Allow main loop to pause
    
    print("\n>>> EXECUTING HANDSHAKE POSE <<<")
    print("Reading physical joint positions for Handshake sequence...")
    initial_ticks = {}
    for motor_id in HANDSHAKE_TICKS.keys():
        tick, result, error = g_packetHandler.read2ByteTxRx(g_portHandler, motor_id, 36)
        if result == COMM_SUCCESS:
            initial_ticks[motor_id] = tick
        else:
            initial_ticks[motor_id] = ROBOT_CONFIG[motor_id]["offset"]
            
    print("Synchronously interpolating to Handshake Pose...")
    steps = int(duration * 100)
    for i in range(1, steps + 1):
        start_t = time.perf_counter()
        t = i / float(steps)
        ease = (1 - math.cos(t * math.pi)) / 2.0
        
        g_groupSyncWrite.clearParam()
        for m_id, target_tick in HANDSHAKE_TICKS.items():
            current_tick = int(initial_ticks[m_id] + ease * (target_tick - initial_ticks[m_id]))
            g_groupSyncWrite.addParam(m_id, [DXL_LOBYTE(current_tick), DXL_HIBYTE(current_tick)])
            
        g_groupSyncWrite.txPacket()
        
        elapsed = time.perf_counter() - start_t
        if elapsed < 0.01:
            time.sleep(0.01 - elapsed)
            
    print("--- Handshake Pose Reached! Shaking hand... ---")
    for pump in range(2):
        for i in range(1, 21):
            t = i / 20.0
            tick = int(1000 + t * (965 - 1000))
            g_packetHandler.write2ByteTxRx(g_portHandler, 1, 30, tick)
            time.sleep(0.015)
        for i in range(1, 21):
            t = i / 20.0
            tick = int(965 + t * (1000 - 965))
            g_packetHandler.write2ByteTxRx(g_portHandler, 1, 30, tick)
            time.sleep(0.015)
            
    print(f"--- Holding Handshake Pose for {hold_time}s... ---")
    time.sleep(hold_time)
    
    print("Returning smoothly to Standing Pose...")
    lh_hip, lh_knee, lh_ankle = compute_human_ik(0.23, 0.0)
    rh_hip, rh_knee, rh_ankle = compute_human_ik(0.23, 0.0)
    standing_radians = {
        1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0, 5: 0.0, 6: 0.0,
        7: 0.0, 8: 0.0,
        9: 0.0, 11: rh_hip, 13: -rh_knee, 15: rh_ankle, 17: 0.0,
        10: 0.0, 12: -lh_hip, 14: -lh_knee, 16: lh_ankle, 18: 0.0
    }
    standing_ticks = {m_id: rad_to_tick(m_id, standing_radians[m_id]) for m_id in ROBOT_CONFIG.keys()}
    
    for i in range(1, steps + 1):
        start_t = time.perf_counter()
        t = i / float(steps)
        ease = (1 - math.cos(t * math.pi)) / 2.0
        
        g_groupSyncWrite.clearParam()
        for m_id, start_tick in HANDSHAKE_TICKS.items():
            target_tick = standing_ticks[m_id]
            current_tick = int(start_tick + ease * (target_tick - start_tick))
            g_groupSyncWrite.addParam(m_id, [DXL_LOBYTE(current_tick), DXL_HIBYTE(current_tick)])
            
        g_groupSyncWrite.txPacket()
        
        elapsed = time.perf_counter() - start_t
        if elapsed < 0.01:
            time.sleep(0.01 - elapsed)
            
    global target_h_left, target_h_right, target_shift_left, target_shift_right, target_x_left, target_x_right
    global target_yaw_left, target_yaw_right, target_arm_pitch_left, target_arm_pitch_right
    target_h_left = 0.23
    target_h_right = 0.23
    target_shift_left = 0.0
    target_shift_right = 0.0
    target_x_left = 0.0
    target_x_right = 0.0
    target_yaw_left = 0.0
    target_yaw_right = 0.0
    target_arm_pitch_left = 0.0
    target_arm_pitch_right = 0.0
    
    is_paused = False
    print("--- Back in Standing Pose. Control resumed. ---\n")
def weight_lift_stance():
    global is_paused, target_h_left, target_h_right, target_shift_left, target_shift_right, target_x_left, target_x_right
    global target_yaw_left, target_yaw_right, target_arm_pitch_left, target_arm_pitch_right

    is_paused = True
    time.sleep(0.05)
    
    print("\n>>> EXECUTING WEIGHT LIFTING STANCE <<<")
    target_ticks = {
        1: 1023, 2: 212, 3: 213, 4: 512,
        5: 194, 6: 507, 7: 677, 8: 658,
        9: 498, 10: 402, 11: 526, 12: 491,
        13: 513, 14: 511, 15: 522, 16: 498,
        17: 490, 18: 526
    }
    
    print("Reading current positions...")
    initial_ticks = {}
    for m_id in target_ticks.keys():
        tick, result, error = g_packetHandler.read2ByteTxRx(g_portHandler, m_id, 36)
        if result == COMM_SUCCESS:
            initial_ticks[m_id] = tick
        else:
            initial_ticks[m_id] = ROBOT_CONFIG[m_id]["offset"]
            
    steps = 150 # 1.5 seconds smooth transition
    for i in range(1, steps + 1):
        start_t = time.perf_counter()
        t = i / float(steps)
        ease = (1 - math.cos(t * math.pi)) / 2.0
        
        g_groupSyncWrite.clearParam()
        for m_id, t_tick in target_ticks.items():
            current_tick = int(initial_ticks[m_id] + ease * (t_tick - initial_ticks[m_id]))
            g_groupSyncWrite.addParam(m_id, [DXL_LOBYTE(current_tick), DXL_HIBYTE(current_tick)])
            
        g_groupSyncWrite.txPacket()
        
        elapsed = time.perf_counter() - start_t
        if elapsed < 0.01:
            time.sleep(0.01 - elapsed)
            
    print("--- Stance Reached! Holding for 5 seconds... ---")
    time.sleep(5.0)
    
    print("\nReturning smoothly to Standing Pose...")
    lh_hip, lh_knee, lh_ankle = compute_human_ik(0.23, 0.0)
    rh_hip, rh_knee, rh_ankle = compute_human_ik(0.23, 0.0)
    standing_radians = {
        1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0, 5: 0.0, 6: 0.0,
        7: 0.0, 8: 0.0,
        9: 0.0, 11: rh_hip, 13: -rh_knee, 15: rh_ankle, 17: 0.0,
        10: 0.0, 12: -lh_hip, 14: -lh_knee, 16: lh_ankle, 18: 0.0
    }
    standing_ticks = {m_id: rad_to_tick(m_id, standing_radians[m_id]) for m_id in ROBOT_CONFIG.keys()}
    
    steps = 200  # 2.0 seconds smooth rise back to standing
    for i in range(1, steps + 1):
        start_t = time.perf_counter()
        t = i / float(steps)
        ease = (1 - math.cos(t * math.pi)) / 2.0
        
        g_groupSyncWrite.clearParam()
        for m_id, start_tick in target_ticks.items():
            t_tick = standing_ticks[m_id]
            current_tick = int(start_tick + ease * (t_tick - start_tick))
            g_groupSyncWrite.addParam(m_id, [DXL_LOBYTE(current_tick), DXL_HIBYTE(current_tick)])
            
        g_groupSyncWrite.txPacket()
        
        elapsed = time.perf_counter() - start_t
        if elapsed < 0.01:
            time.sleep(0.01 - elapsed)

    target_h_left = 0.23
    target_h_right = 0.23
    target_shift_left = 0.0
    target_shift_right = 0.0
    target_x_left = 0.0
    target_x_right = 0.0
    target_yaw_left = 0.0
    target_yaw_right = 0.0
    target_arm_pitch_left = 0.0
    target_arm_pitch_right = 0.0
    
    is_paused = False
    print("--- Back in Standing Pose. Control resumed. ---\n")
def push_up(num_reps=15, elbow_bend_deg=120.0, rep_duration=1.2):
    global is_paused
    is_paused = True
    time.sleep(0.05) # Allow main loop to pause
    
    print("\n>>> EXECUTING KINEMATIC PUSH-UP EXERCISE <<<")
    print(f"Parameters: Reps={num_reps}, Elbow Flexion={elbow_bend_deg}°, Rep Duration={rep_duration}s")
    
    # =========================================================================
    # KINEMATIC DESIGN (Cosine Rule & Tangent Rule)
    # =========================================================================
    # 1. Cosine Rule: Calculate how much the chest lowers (Delta_h) when bending elbow
    # Model arm as two equal links: L_upper and L_fore
    L_upper = 0.10  # 10 cm upper arm link
    L_fore  = 0.10  # 10 cm forearm link
    L_body  = 0.28  # 28 cm effective pivot distance from ankle to shoulder support
    
    # Check physical symmetrical servo limits for elbow flexion first!
    max_sym_elbow_ticks = min(PUSHUP_START_TICKS[5], 1023 - PUSHUP_START_TICKS[6])
    requested_elbow_ticks = int(math.radians(elbow_bend_deg) * TICKS_PER_RADIAN)
    if requested_elbow_ticks > max_sym_elbow_ticks:
        print(f" [Kinematic Limit] Requested {elbow_bend_deg}° ({requested_elbow_ticks} ticks) exceeds servo limit! Clamping to max physical bend: {max_sym_elbow_ticks} ticks (~{math.degrees(max_sym_elbow_ticks / TICKS_PER_RADIAN):.1f}°).")
        delta_ticks_elbow = max_sym_elbow_ticks
        theta_rad = delta_ticks_elbow / TICKS_PER_RADIAN
    else:
        delta_ticks_elbow = requested_elbow_ticks
        theta_rad = math.radians(elbow_bend_deg)
        
    # Distance from shoulder to hand when arm is straight (theta = 0) vs bent by theta
    D_start = L_upper + L_fore
    # Cosine rule: D_bent^2 = L_upper^2 + L_fore^2 - 2*L_upper*L_fore*cos(pi - theta)
    #                       = L_upper^2 + L_fore^2 + 2*L_upper*L_fore*cos(theta)
    D_bent = math.sqrt(L_upper**2 + L_fore**2 + 2 * L_upper * L_fore * math.cos(theta_rad))
    delta_h_chest = D_start - D_bent
    
    # 2. Tangent Rule: Calculate required ankle pitch angle change to keep foot flat
    # X_body is the horizontal ground distance from ankle pivot to hand support
    X_body = math.sqrt(max(0.001, L_body**2 - D_start**2))
    alpha_start_rad = math.atan(D_start / X_body)
    alpha_bent_rad  = math.atan(D_bent / X_body)
    delta_alpha_ankle_rad = alpha_start_rad - alpha_bent_rad
    
    # Scale ankle angle by empirical contact gain (to account for foot sole geometry)
    ankle_contact_gain = 1.5  
    effective_ankle_rad = delta_alpha_ankle_rad * ankle_contact_gain
    
    # Convert kinematic radian deltas to Dynamixel AX-12A ticks
    delta_ticks_ankle = int(effective_ankle_rad * TICKS_PER_RADIAN)
    delta_ticks_shoulder = int(delta_ticks_elbow * 0.30) # Increased compensation for deeper dip
    delta_ticks_shoulder_roll = int(delta_ticks_elbow * 0.45) # Increased outward abduction to keep fingers stationary
    
    print(f" [Cosine Rule] Chest Lowering (Δh): {delta_h_chest*100:.2f} cm (D_start={D_start*100:.1f}cm -> D_bent={D_bent*100:.1f}cm)")
    print(f" [Tangent Rule] Ankle Tilt Angle (Δα): {math.degrees(effective_ankle_rad):.2f}°")
    print(f" [Dynamixel Targets] ΔElbow={delta_ticks_elbow}, ΔAnkle={delta_ticks_ankle}, ΔShoulderPitch={delta_ticks_shoulder}, ΔShoulderRoll={delta_ticks_shoulder_roll}")
    
    # Dynamically generate bottom position ticks using the kinematic results
    bottom_ticks = dict(PUSHUP_START_TICKS)
    bottom_ticks[1] = min(1023, PUSHUP_START_TICKS[1] + delta_ticks_shoulder)       # Shoulder Pitch R extends
    bottom_ticks[2] = max(0,    PUSHUP_START_TICKS[2] - delta_ticks_shoulder)       # Shoulder Pitch L extends
    bottom_ticks[3] = min(1023, PUSHUP_START_TICKS[3] + delta_ticks_shoulder_roll)  # Shoulder Roll R rotates outward
    bottom_ticks[4] = max(0,    PUSHUP_START_TICKS[4] - delta_ticks_shoulder_roll)  # Shoulder Roll L rotates outward
    bottom_ticks[5] = max(0,    PUSHUP_START_TICKS[5] - delta_ticks_elbow)          # Elbow R bent inward
    bottom_ticks[6] = min(1023, PUSHUP_START_TICKS[6] + delta_ticks_elbow)          # Elbow L bent inward
    bottom_ticks[15] = min(1023, PUSHUP_START_TICKS[15] + delta_ticks_ankle)        # Ankle Pitch R tilts forward (keep foot flat)
    bottom_ticks[16] = max(0,    PUSHUP_START_TICKS[16] - delta_ticks_ankle)        # Ankle Pitch L tilts forward (keep foot flat)
    
    print("\nReading physical joint positions for Push-Up sequence...")
    initial_ticks = {}
    for motor_id in PUSHUP_START_TICKS.keys():
        tick, result, error = g_packetHandler.read2ByteTxRx(g_portHandler, motor_id, 36)
        if result == COMM_SUCCESS:
            initial_ticks[motor_id] = tick
        else:
            initial_ticks[motor_id] = ROBOT_CONFIG[motor_id]["offset"]
            
    print("Phase 1: Simultaneously raising hands and falling into Push-Up Plank Position...")
    steps = 200  # 2.0 seconds smooth fall into plank
    for i in range(1, steps + 1):
        start_t = time.perf_counter()
        t = i / float(steps)
        ease = (1 - math.cos(t * math.pi)) / 2.0
        
        g_groupSyncWrite.clearParam()
        for m_id, target_tick in PUSHUP_START_TICKS.items():
            current_tick = int(initial_ticks[m_id] + ease * (target_tick - initial_ticks[m_id]))
            g_groupSyncWrite.addParam(m_id, [DXL_LOBYTE(current_tick), DXL_HIBYTE(current_tick)])
            
        g_groupSyncWrite.txPacket()
        
        elapsed = time.perf_counter() - start_t
        if elapsed < 0.01:
            time.sleep(0.01 - elapsed)
            
    print("--- Push-Up Starting Pose Reached! Stabilizing... ---")
    time.sleep(0.8)
    
    print(f"Phase 2: Executing {num_reps} Push-Up repetitions (Bending elbows & pitching ankles)...")
    rep_steps = int(rep_duration * 100)
    if rep_steps <= 0: rep_steps = 1
    
    for r in range(1, num_reps + 1):
        print(f"  [+] Rep {r}/{num_reps}: Lowering chest (Elbow flex & ankle tilt)...")
        for i in range(1, rep_steps + 1):
            start_t = time.perf_counter()
            t = i / float(rep_steps)
            ease = (1 - math.cos(t * math.pi)) / 2.0
            
            g_groupSyncWrite.clearParam()
            for m_id, start_tick in PUSHUP_START_TICKS.items():
                target_tick = bottom_ticks[m_id]
                current_tick = int(start_tick + ease * (target_tick - start_tick))
                g_groupSyncWrite.addParam(m_id, [DXL_LOBYTE(current_tick), DXL_HIBYTE(current_tick)])
                
            g_groupSyncWrite.txPacket()
            
            elapsed = time.perf_counter() - start_t
            if elapsed < 0.01:
                time.sleep(0.01 - elapsed)
                
        time.sleep(0.2)  # Hold bottom dip
        
        print(f"  [+] Rep {r}/{num_reps}: Pushing back up to plank...")
        for i in range(1, rep_steps + 1):
            start_t = time.perf_counter()
            t = i / float(rep_steps)
            ease = (1 - math.cos(t * math.pi)) / 2.0
            
            g_groupSyncWrite.clearParam()
            for m_id, start_tick in bottom_ticks.items():
                target_tick = PUSHUP_START_TICKS[m_id]
                current_tick = int(start_tick + ease * (target_tick - start_tick))
                g_groupSyncWrite.addParam(m_id, [DXL_LOBYTE(current_tick), DXL_HIBYTE(current_tick)])
                
            g_groupSyncWrite.txPacket()
            
            elapsed = time.perf_counter() - start_t
            if elapsed < 0.01:
                time.sleep(0.01 - elapsed)
                
        time.sleep(0.2)  # Hold plank top
        
    print("\nPhase 3: Returning smoothly to Standing Pose...")
    lh_hip, lh_knee, lh_ankle = compute_human_ik(0.23, 0.0)
    rh_hip, rh_knee, rh_ankle = compute_human_ik(0.23, 0.0)
    standing_radians = {
        1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0, 5: 0.0, 6: 0.0,
        7: 0.0, 8: 0.0,
        9: 0.0, 11: rh_hip, 13: -rh_knee, 15: rh_ankle, 17: 0.0,
        10: 0.0, 12: -lh_hip, 14: -lh_knee, 16: lh_ankle, 18: 0.0
    }
    standing_ticks = {m_id: rad_to_tick(m_id, standing_radians[m_id]) for m_id in ROBOT_CONFIG.keys()}
    
    steps = 200  # 2.0 seconds smooth rise back to standing
    for i in range(1, steps + 1):
        start_t = time.perf_counter()
        t = i / float(steps)
        ease = (1 - math.cos(t * math.pi)) / 2.0
        
        g_groupSyncWrite.clearParam()
        for m_id, start_tick in PUSHUP_START_TICKS.items():
            target_tick = standing_ticks[m_id]
            current_tick = int(start_tick + ease * (target_tick - start_tick))
            g_groupSyncWrite.addParam(m_id, [DXL_LOBYTE(current_tick), DXL_HIBYTE(current_tick)])
            
        g_groupSyncWrite.txPacket()
        
        elapsed = time.perf_counter() - start_t
        if elapsed < 0.01:
            time.sleep(0.01 - elapsed)
            
    global target_h_left, target_h_right, target_shift_left, target_shift_right, target_x_left, target_x_right
    global target_yaw_left, target_yaw_right, target_arm_pitch_left, target_arm_pitch_right
    target_h_left = 0.23
    target_h_right = 0.23
    target_shift_left = 0.0
    target_shift_right = 0.0
    target_x_left = 0.0
    target_x_right = 0.0
    target_yaw_left = 0.0
    target_yaw_right = 0.0
    target_arm_pitch_left = 0.0
    target_arm_pitch_right = 0.0
    
    is_paused = False
    print("--- Back in Standing Pose. Control resumed. ---\n")

def smooth_crouch_transition(start_offset, end_offset, start_pitch, end_pitch, duration=1.0):
    global g_crouch_offset, g_arm_pitch_offset, target_h_left, target_h_right
    steps = int(duration * 50)
    if steps <= 0: steps = 1
    dt = duration / steps
    base_h = 0.23
    
    for i in range(1, steps + 1):
        t = i / float(steps)
        ease = (1 - math.cos(t * math.pi)) / 2.0
        
        g_crouch_offset = start_offset + ease * (end_offset - start_offset)
        g_arm_pitch_offset = start_pitch + ease * (end_pitch - start_pitch)
        
        target_h_left = max(0.14, base_h - g_crouch_offset)
        target_h_right = max(0.14, base_h - g_crouch_offset)
        time.sleep(dt)

def fast_crouch_walk_loop(num_steps=8):
    if num_steps < 1:
        return
        
    print("\n=== STEP 1: Fast Initial Right Step (Crouch Mode) ===")
    print("Stage 1: Pre-Crouch (Double Support)...")
    transition_to(0.22, 0.22, 0.0, 0.0, 0.0, 0.0, 0.35)
    
    print("Stage 2: Left Lateral Shift (Weight on Left)...")
    transition_to(0.22, 0.22, 0.30, 0.30, 0.00, 0.00, 0.35) 
    
    print("Stage 3: Lifting Right Leg...")
    transition_to(0.22, 0.12, 0.30, 0.40, 0.00, 0.00, 0.25)

    print("Stage 4: Fast Initial Right Swing...")
    swing_trajectory_right(0.65)
    
    print("Stage 5: Right Touchdown...")
    h_l, h_r, shift_l, shift_r, x_l, x_r = generate_stage5(RIGHT_STEP_LENGTH)
    transition_to(h_l, h_r, shift_l, shift_r, x_l, x_r, 0.45)

    print("Stage 6: Forward lean (Weight on Right Leg)...")
    h_l, h_r, shift_l, shift_r, x_l, x_r = generate_stage6(RIGHT_STEP_LENGTH)
    transition_to(h_l, h_r, shift_l, shift_r, x_l, x_r, 0.45)

    for i in range(2, num_steps + 1):
        if i % 2 == 0:
            print(f"\n=== STEP {i}: Fast Continuous Left Step (Crouch Mode) ===")
            print("Left Swing...")
            swing_trajectory_left(1.25)
            
            print("Left Touchdown...")
            h_l, h_r, shift_l, shift_r, x_l, x_r = generate_stage8(LEFT_STEP_LENGTH)
            transition_to(h_l, h_r, shift_l, shift_r, x_l, x_r, 0.25)
            
            print("Forward Lean (Weight on Left Leg)...")
            h_l, h_r, shift_l, shift_r, x_l, x_r = generate_stage9(LEFT_STEP_LENGTH)
            transition_to(h_l, h_r, shift_l, shift_r, x_l, x_r, 0.25)
        else:
            print(f"\n=== STEP {i}: Fast Continuous Right Step (Crouch Mode) ===")
            print("Continuous Right Swing...")
            swing_trajectory_right_continuous(0.60)
            
            print("Right Touchdown...")
            h_l, h_r, shift_l, shift_r, x_l, x_r = generate_stage5(RIGHT_STEP_LENGTH)
            transition_to(h_l, h_r, shift_l, shift_r, x_l, x_r, 0.25)
            
            print("Forward Lean (Weight on Right Leg)...")
            h_l, h_r, shift_l, shift_r, x_l, x_r = generate_stage6(RIGHT_STEP_LENGTH)
            transition_to(h_l, h_r, shift_l, shift_r, x_l, x_r, 0.25)
            
    print("\n--- Walk Complete. Aligning Trailing Leg Forward... ---")
    if num_steps % 2 == 1:
        print("Committing Weight entirely to Right Leg...")
        transition_to(0.22, 0.22, -0.30, -0.30, target_x_left, target_x_right, 0.25)
        print("Swinging Left Leg Forward to Align...")
        swing_left_forward_align(0.65)
    else:
        print("Committing Weight entirely to Left Leg...")
        transition_to(0.22, 0.22, 0.30, 0.30, target_x_left, target_x_right, 0.25)
        print("Swinging Right Leg Forward to Align...")
        swing_right_forward_align(0.65)
        
    print("Resetting to crouch standing pose...")
    transition_to(0.23, 0.23, 0.0, 0.0, 0.0, 0.0, 0.5)

def crouch_walk_sequence(num_steps=8):
    global g_crouch_offset, g_arm_pitch_offset
    print("\n>>> EXECUTING STEALTH CROUCH WALK (ULTRA-SMOOTH & HIGH SPEED) <<<")
    print("Smoothly lowering into deep stealth crouch (knees bent, arms in ninja guard)...")
    
    smooth_crouch_transition(0.0, 0.05, 0.0, 0.3, duration=1.0)
        
    print(f"\n[+] Walking {num_steps} steps at HIGH SPEED in stealth crouch mode...")
    fast_crouch_walk_loop(num_steps=num_steps)
    
    print("\nSmoothly rising back up from stealth crouch to normal standing height...")
    smooth_crouch_transition(0.05, 0.0, 0.3, 0.0, duration=1.0)
        
    g_crouch_offset = 0.0
    g_arm_pitch_offset = 0.0
    print("--- Crouch Walk Complete! ---\n")

def moonwalk(num_steps=6):
    global target_arm_pitch_left, target_arm_pitch_right
    print("\n>>> EXECUTING THE MOONWALK (BACKWARD GLIDE) <<<")
    print("Stage 1: Pre-crouch and shift weight to Left Leg...")
    transition_to(0.22, 0.22, -0.008, -0.008, 0.00, 0.00, 0.5)
    transition_to(0.22, 0.22, 0.14, 0.14, -0.008, -0.008, 0.5)
    
    cur_x_l = -0.008
    cur_x_r = -0.008
    stride = 0.035  # Compact 3.5cm glide for maximum balance
    bias = -0.008   # 8mm backward CoM bias to keep weight off toes
    
    for step in range(1, num_steps + 1):
        if step % 2 == 1:
            print(f"\n=== MOONWALK STEP {step}: Left Support Pushes Back, Right Glides Behind ===")
            target_xl = bias + stride
            target_xr = bias - stride
            
            steps_loop = 45
            dt = 0.9 / steps_loop
            for i in range(1, steps_loop + 1):
                t = i / float(steps_loop)
                ease = (1 - math.cos(t * math.pi)) / 2.0
                
                x_l = cur_x_l + ease * (target_xl - cur_x_l)
                x_r = cur_x_r + ease * (target_xr - cur_x_r)
                
                h_l = 0.22
                h_r = 0.22 - 0.02 * math.sin(math.pi * t)
                
                target_arm_pitch_left = 0.15 * math.sin(math.pi * t)
                target_arm_pitch_right = -0.15 * math.sin(math.pi * t)
                
                set_targets(h_l, h_r, 0.14, 0.14, x_l, x_r, dt)
                
            cur_x_l = target_xl
            cur_x_r = target_xr
            target_arm_pitch_left = 0.0
            target_arm_pitch_right = 0.0
            
            print("Pop weight over to Right Leg...")
            transition_to(0.22, 0.22, -0.14, -0.14, cur_x_l, cur_x_r, 0.75)
        else:
            print(f"\n=== MOONWALK STEP {step}: Right Support Pushes Back, Left Glides Behind ===")
            target_xr = bias + stride
            target_xl = bias - stride
            
            steps_loop = 45
            dt = 0.9 / steps_loop
            for i in range(1, steps_loop + 1):
                t = i / float(steps_loop)
                ease = (1 - math.cos(t * math.pi)) / 2.0
                
                x_r = cur_x_r + ease * (target_xr - cur_x_r)
                x_l = cur_x_l + ease * (target_xl - cur_x_l)
                
                h_r = 0.22
                h_l = 0.22 - 0.02 * math.sin(math.pi * t)
                
                target_arm_pitch_left = -0.15 * math.sin(math.pi * t)
                target_arm_pitch_right = 0.15 * math.sin(math.pi * t)
                
                set_targets(h_l, h_r, -0.14, -0.14, x_l, x_r, dt)
                
            cur_x_l = target_xl
            cur_x_r = target_xr
            target_arm_pitch_left = 0.0
            target_arm_pitch_right = 0.0
            
            print("Pop weight over to Left Leg...")
            transition_to(0.22, 0.22, 0.14, 0.14, cur_x_l, cur_x_r, 0.75)
            
    print("\n--- Moonwalk Complete! Centering feet and rising... ---")
    transition_to(0.22, 0.22, 0.00, 0.00, -0.008, -0.008, 0.6)
    transition_to(0.23, 0.23, 0.00, 0.00, 0.00, 0.00, 0.7)
    print("--- Back in Standing Pose. ---\n")


def kick_swing_forward(duration=0.15, target_x_r=0.11):
    steps = int(duration * 100)
    if steps <= 0: steps = 1
    dt = duration / steps
    h_l_final, h_r_final, _, _, _, _ = generate_stage5(RIGHT_STEP_LENGTH)
    
    for i in range(1, steps + 1):
        t = i / float(steps)
        x_r = t * target_x_r
        x_l = 0.0
        h_linear = 0.17 + t * (0.15 - 0.17)
        h_r = h_linear - 0.03 * math.sin(math.pi * t)
        com_lateral_sway = 0.03 * math.sin(math.pi * t)
        shift_l = 0.30
        shift_r = (0.40 - t * 0.05) + com_lateral_sway
        x_com = 2.0 + 2.0 * t
        com_bump = 0.015625 * ((x_com - 4.0) ** 2) * (x_com - 2.0)
        h_l = h_l_final + 0.5 * com_bump
        set_targets(h_l, h_r, shift_l, shift_r, x_l, x_r, dt)

def kick_swing_backward(duration=0.18, start_x_r=0.11):
    steps = int(duration * 100)
    if steps <= 0: steps = 1
    dt = duration / steps
    h_l_final, h_r_final, _, _, _, _ = generate_stage5(RIGHT_STEP_LENGTH)
    
    for i in range(1, steps + 1):
        t = i / float(steps)
        x_r = (1.0 - t) * start_x_r
        x_l = 0.0
        h_linear = 0.15 + t * (0.17 - 0.15)
        h_r = h_linear - 0.03 * math.sin(math.pi * t)
        com_lateral_sway = 0.03 * math.sin(math.pi * t)
        shift_l = 0.30
        shift_r = (0.35 + t * 0.05) + com_lateral_sway
        x_com = 4.0 - 2.0 * t
        com_bump = 0.015625 * ((x_com - 4.0) ** 2) * (x_com - 2.0)
        h_l = h_l_final + 0.5 * com_bump
        set_targets(h_l, h_r, shift_l, shift_r, x_l, x_r, dt)

def kick():
    print("\n>>> EXECUTING DYNAMIC KICK (HIGH SPEED) <<<")
    
    print("Stage 1: Pre-crouch and prepare weight transfer...")
    transition_to(0.22, 0.22, 0.00, 0.00, 0.00, 0.00, 0.25)
    
    print("Stage 2: Shifting weight entirely to Left Leg...")
    transition_to(0.22, 0.22, 0.30, 0.30, 0.00, 0.00, 0.30)
    
    print("Stage 3: Lifting Right Leg (old_motion_file values)...")
    transition_to(0.22, 0.17, 0.30, 0.40, 0.00, 0.00, 0.25)
    
    print("Stage 4: STRIKE! Fast forward swing...")
    kick_swing_forward(0.15, 0.11)
    
    print("Holding extended kick...")
    time.sleep(0.15)
    
    print("Stage 5: Fast retraction back under hip...")
    kick_swing_backward(0.18, 0.11)
    
    print("Stage 6: Planting foot on ground...")
    transition_to(0.22, 0.22, 0.30, 0.30, 0.00, 0.00, 0.20)
    
    print("Stage 7: Recentering weight and standing up...")
    transition_to(0.23, 0.23, 0.00, 0.00, 0.00, 0.00, 0.30)
    
    print("--- Kick Complete! ---\n")

def go_to_shutdown_pose():
    global is_shutting_down
    is_shutting_down = True
    time.sleep(0.05) # Allow main loop to pause
    
    print("\nReading physical joint positions for shutdown sequence...")
    initial_ticks = {}
    for motor_id in SHUTDOWN_TICKS.keys():
        tick, result, error = g_packetHandler.read2ByteTxRx(g_portHandler, motor_id, 36)
        if result == COMM_SUCCESS:
            initial_ticks[motor_id] = tick
        else:
            initial_ticks[motor_id] = SHUTDOWN_TICKS[motor_id]
            
    print("Synchronously interpolating to Shutdown Pose...")
    duration = 2.0
    steps = int(duration * 100)
    for i in range(1, steps + 1):
        start_t = time.perf_counter()
        t = i / float(steps)
        ease = (1 - math.cos(t * math.pi)) / 2.0
        
        g_groupSyncWrite.clearParam()
        for m_id, target_tick in SHUTDOWN_TICKS.items():
            current_tick = int(initial_ticks[m_id] + ease * (target_tick - initial_ticks[m_id]))
            g_groupSyncWrite.addParam(m_id, [DXL_LOBYTE(current_tick), DXL_HIBYTE(current_tick)])
            
        g_groupSyncWrite.txPacket()
        
        elapsed = time.perf_counter() - start_t
        if elapsed < 0.01:
            time.sleep(0.01 - elapsed)
            
    print("Robot safely parked. Powering down torque...")
    for m_id in SHUTDOWN_TICKS.keys():
        g_packetHandler.write1ByteTxRx(g_portHandler, m_id, 24, 0)
    
    g_portHandler.closePort()
    print("Shutdown complete. Exiting.\n")
    os._exit(0)

current_command = None

def keyboard_listener_thread():
    global current_command, abort_walk

    time.sleep(0.5)

    while True:
        ch = None
        if is_windows:
            if msvcrt.kbhit():
                ch = msvcrt.getch().decode(errors="ignore").lower()
        else:
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            try:
                tty.setcbreak(sys.stdin.fileno())
                if select.select([sys.stdin], [], [], 0.01)[0]:
                    ch = sys.stdin.read(1).lower()
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
                
        if ch:

            if ch == 'k':
                print("\n>>> EMERGENCY ABORT TRIGGERED <<<")
                abort_walk = True
                time.sleep(0.15)
                abort_walk = False

                print(">>> Safely planting both feet on the ground... <<<")
                transition_to(0.23,0.23,0,0,0,0,1.5)

                print("\n>>> SHUTDOWN <<<")
                go_to_shutdown_pose()

            elif ch == 'a':
                current_command = "turn_left"

            elif ch == 'd':
                current_command = "turn_right"

            elif ch == 'e':
                current_command = "dab"

            elif ch == 'f':
                current_command = "kick"

            elif ch == 'h':
                current_command = "handshake"

            elif ch == 'c':
                current_command = "crouch_walk"

            elif ch == 'm':
                current_command = "moonwalk"

            elif ch == 'p':
                current_command = "push_up"
            elif ch == 'l':
                current_command = "weight_lift"


            elif ch == 'w':
                print("\n[!] Enter distance to walk (in meters) and press ENTER:")
                try:
                    dist_str = input("> ")
                    distance = float(dist_str)
                    num_steps = max(1, int(distance / RIGHT_STEP_LENGTH))
                    print(f"\n[+] Walking {distance}m ({num_steps} steps)...")
                    current_command = ('walk_dist', num_steps)
                except ValueError:
                    print("\n[!] Invalid distance. Walk cancelled.")
            elif ch == '\r' or ch == '\n':
                current_command = "walk"
            elif ch == ' ':
                print("\n>>> SPACEBAR PRESSED: INSTANT KILL SWITCH <<<")
                global is_shutting_down
                is_shutting_down = True
                print("Disabling Torque on all motors...")
                for motor_id in ROBOT_CONFIG.keys():
                    g_packetHandler.write1ByteTxRx(
                     g_portHandler,
                        motor_id,
                        24,   # Torque Enable address
                        0     # Disable torque
                    )
                print("Torque disabled. Exiting instantly.")
                os._exit(0)

        if is_windows:
            time.sleep(0.01)

def execute_turn(total_angle, dir_sign):
    global target_yaw_left, target_yaw_right, target_arm_pitch_left, target_arm_pitch_right
    direction_str = "LEFT" if dir_sign == -1 else "RIGHT"
    print(f"\n=== EXECUTING TURN: {direction_str} (Total Angle: {total_angle}) ===")
    
    n_full_steps = int(total_angle // MAX_TURN_STEP_ANGLE)
    remainder = total_angle % MAX_TURN_STEP_ANGLE
    
    steps = [MAX_TURN_STEP_ANGLE] * n_full_steps
    if remainder > 0.01:
        steps.append(remainder)
        
    for i, step_angle in enumerate(steps):
        print(f"\n--- Turn Step {i+1}/{len(steps)}: {step_angle:.2f} rad ---")
        
        print("Stage 1: Pre-Crouch (Double Support)...")
        transition_to(0.22, 0.22, 0.0, 0.0, 0.0, 0.0, 0.7)
        
        if dir_sign == -1: # Left Turn
            print("Stage 2: Right Lateral Shift (Weight on Right)...")
            transition_to(0.22, 0.22, -0.38, -0.38, 0.00, 0.00, 0.7)
            
            print("Stage 3: Lifting Left Leg...") 
            transition_to(TURN_LIFT_HEIGHT, 0.22, -0.48, -0.38, 0.00, 0.00, 0.7)
            
            print(f"Turning Left Leg (Outward by {step_angle:.2f})...")
            transition_yaw(-step_angle, target_yaw_right, 0.7)
            
            print("Placing Left Leg on ground...")
            transition_to(0.22, 0.22, -0.48, -0.38, 0.00, 0.00, 0.7)
            
            print("Left Lateral Shift (Weight on Left)...")
            transition_to(0.22, 0.22, 0.30, 0.30, 0.00, 0.00, 0.7)
            
            print("Lifting Right Leg...")
            transition_to(0.22, TURN_LIFT_HEIGHT, 0.30, 0.40, 0.00, 0.00, 0.7)
            
            print("Swinging Pelvis (Untwisting Left Leg)...")
            transition_yaw(0.0, target_yaw_right, 0.7)
            
            print("Placing Right Leg on ground...")
            transition_to(0.22, 0.22, 0.30, 0.40, 0.00, 0.00, 0.7)
            
        else: # Right Turn
            print("Stage 2: Left Lateral Shift (Weight on Left)...")
            transition_to(0.22, 0.22, 0.30, 0.30, 0.00, 0.00, 0.7) 
            
            print("Stage 3: Lifting Right Leg...")
            transition_to(0.22, TURN_LIFT_HEIGHT, 0.30, 0.40, 0.00, 0.00, 0.7)
            
            print(f"Turning Right Leg (Outward by {step_angle:.2f})...")
            transition_yaw(target_yaw_left, step_angle, 0.7)
            
            print("Placing Right Leg on ground...")
            transition_to(0.22, 0.22, 0.30, 0.40, 0.00, 0.00, 0.7)
            
            print("Right Lateral Shift (Weight on Right)...")
            transition_to(0.22, 0.22, -0.38, -0.38, 0.00, 0.00, 0.7)
            
            print("Lifting Left Leg...")
            transition_to(TURN_LIFT_HEIGHT, 0.22, -0.48, -0.38, 0.00, 0.00, 0.7)
            
            print("Swinging Pelvis (Untwisting Right Leg)...")
            transition_yaw(target_yaw_left, 0.0, 0.7)
            
            print("Placing Left Leg on ground...")
            transition_to(0.22, 0.22, -0.48, -0.38, 0.00, 0.00, 0.7)
            
        print("Recentering weight...")
        transition_to(0.23, 0.23, 0.0, 0.0, 0.0, 0.0, 0.7)
def state_machine_thread():
    global current_command
    time.sleep(1.0)
    print("[ENTER: Walk | 'q': Left | 'e': Right | 's': Crouch Walk | 'm': Moonwalk | 'p': Push-Up | 'd': Dab | 'f': Kick | 'h': Handshake | 'k': Safe Stop | SPACE: Kill]")
    while True:
        while current_command is None:
            time.sleep(0.05)
            
        cmd = current_command
        current_command = None
        
        try:
            if isinstance(cmd, tuple) and cmd[0] == 'walk_dist':
                walk_sequence(num_steps=cmd[1])
            elif cmd == 'walk':
                walk_sequence(num_steps=10)
            elif cmd == 'turn_left':
                execute_turn(DEFAULT_TURN_LEFT_ANGLE, -1)
            elif cmd == 'turn_right':
                execute_turn(DEFAULT_TURN_RIGHT_ANGLE, 1)
            elif cmd == 'dab':
                dab(1.0, 2.5)
            elif cmd == 'kick':
                kick()
            elif cmd == 'handshake':
                handshake()
            elif cmd == 'crouch_walk':
                crouch_walk_sequence(num_steps=8)
            elif cmd == 'moonwalk':
                moonwalk(num_steps=6)
            elif cmd == 'push_up':
                push_up(num_reps=5)
            elif cmd == 'weight_lift':
                weight_lift_stance()


        except AbortWalk:
            print("Motion Aborted.")
            pass

# =============================================================================
# 4. FLAWLESS 3-DOF GEOMETRIC IK
# =============================================================================
def compute_human_ik(height, x_offset):
    y = height
    x = x_offset
    d = math.sqrt(x*x + y*y)
    
    max_len = L1 + L2 - 0.00001
    d = min(d, max_len)

    cos_knee = (d*d - L1*L1 - L2*L2) / (2 * L1 * L2)
    cos_knee = max(min(cos_knee, 1.0), -1.0)
    knee_raw = math.acos(cos_knee)

    knee_geom = -knee_raw
    hip_geom = math.atan2(x, y) + math.atan2(L2 * math.sin(knee_raw), L1 + L2 * math.cos(knee_raw))
    ankle_geom = -(hip_geom + knee_geom)

    return hip_geom, knee_geom, ankle_geom

def tick_to_rad(motor_id, tick):
    config = ROBOT_CONFIG[motor_id]
    tick_delta = tick - config["offset"]
    radian = tick_delta / (TICKS_PER_RADIAN * config["dir"])
    return radian

# =============================================================================
# 5. MAIN CONTROL LOOP (GroupSyncWrite)
# =============================================================================
def main():
    global target_h_left, target_h_right, target_shift_left, target_shift_right, target_x_left, target_x_right

    # Initialize Dynamixel Port and Packet Handlers
    portHandler = PortHandler(DEVICENAME)
    packetHandler = PacketHandler(PROTOCOL_VERSION)
    groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_AX_GOAL_POSITION, LEN_AX_GOAL_POSITION)

    global g_portHandler, g_packetHandler, g_groupSyncWrite
    g_portHandler = portHandler
    g_packetHandler = packetHandler
    g_groupSyncWrite = groupSyncWrite

    if not portHandler.openPort():
        print(f"Failed to open port {DEVICENAME}. Check connection and permissions.")
        quit()
    if not portHandler.setBaudRate(BAUDRATE):
        print(f"Failed to change the baudrate to {BAUDRATE}")
        quit()

    print("\nEnabling Torque on all motors...")
    for motor_id in ROBOT_CONFIG.keys():
        packetHandler.write1ByteTxRx(portHandler, motor_id, 24, 1)

    print("\nReading physical joint positions for smooth initialization...")
    initial_radians = {}
    for motor_id in ROBOT_CONFIG.keys():
        tick, result, error = packetHandler.read2ByteTxRx(portHandler, motor_id, 36)
        if result == COMM_SUCCESS:
            initial_radians[motor_id] = tick_to_rad(motor_id, tick)
        else:
            print(f"Warning: Failed to read motor {motor_id}, defaulting to 0.0")
            initial_radians[motor_id] = 0.0

    # Calculate exact standing IK pose
    lh_hip, lh_knee, lh_ankle = compute_human_ik(0.23, 0.0)
    rh_hip, rh_knee, rh_ankle = compute_human_ik(0.23, 0.0)
    standing_radians = {
        1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0, 5: 0.0, 6: 0.0,
        7: 0.0, 8: 0.0,
        9: 0.0, 11: rh_hip, 13: -rh_knee, 15: rh_ankle, 17: 0.0,
        10: 0.0, 12: -lh_hip, 14: -lh_knee, 16: lh_ankle, 18: 0.0
    }

    print("--- Initializing: Synchronously interpolating to Standing Pose... ---")
    duration = 2.0
    steps = int(duration * 100)
    for i in range(1, steps + 1):
        start_t = time.perf_counter()
        t = i / float(steps)
        ease = (1 - math.cos(t * math.pi)) / 2.0
        
        groupSyncWrite.clearParam()
        for m_id in ROBOT_CONFIG.keys():
            current_rad = initial_radians[m_id] + ease * (standing_radians[m_id] - initial_radians[m_id])
            tick = rad_to_tick(m_id, current_rad)
            groupSyncWrite.addParam(m_id, [DXL_LOBYTE(tick), DXL_HIBYTE(tick)])
        
        groupSyncWrite.txPacket()
        elapsed = time.perf_counter() - start_t
        if elapsed < 0.01:
            time.sleep(0.01 - elapsed)

    print("--- Robot is Ready! ---")

    # Start the state machine and keyboard listener NOW that the robot is safely standing
    threading.Thread(target=keyboard_listener_thread, daemon=True).start()
    threading.Thread(target=state_machine_thread, daemon=True).start()

    current_h_left = 0.23
    current_h_right = 0.23
    current_shift_left = 0.0
    current_shift_right = 0.0
    current_x_left = 0.0

    current_x_right = 0.0
    current_yaw_left = 0.0
    current_yaw_right = 0.0
    current_arm_pitch_left = 0.0
    current_arm_pitch_right = 0.0
    
    print("10-DOF Humanoid Step Controller Running via Dynamixel SDK!")
    
    # Target frequency: 100Hz
    loop_time = 0.01 

    while True:
        if is_shutting_down or is_paused:
            current_h_left = 0.23
            current_h_right = 0.23
            current_shift_left = 0.0
            current_shift_right = 0.0
            current_x_left = 0.0
            current_x_right = 0.0
            current_yaw_left = 0.0
            current_yaw_right = 0.0
            current_arm_pitch_left = 0.0
            current_arm_pitch_right = 0.0
            time.sleep(0.05)
            continue
            
        start_time = time.perf_counter()

        alpha = 0.05
        current_h_left += (target_h_left - current_h_left) * alpha
        current_h_right += (target_h_right - current_h_right) * alpha
        current_shift_left += (target_shift_left - current_shift_left) * alpha
        current_shift_right += (target_shift_right - current_shift_right) * alpha
        current_x_left += (target_x_left - current_x_left) * alpha
        current_x_right += (target_x_right - current_x_right) * alpha

        current_yaw_left += (target_yaw_left - current_yaw_left) * alpha
        current_yaw_right += (target_yaw_right - current_yaw_right) * alpha
        current_arm_pitch_left += ((target_arm_pitch_left + g_arm_pitch_offset) - current_arm_pitch_left) * alpha
        current_arm_pitch_right += ((target_arm_pitch_right + g_arm_pitch_offset) - current_arm_pitch_right) * alpha

        lh_hip, lh_knee, lh_ankle = compute_human_ik(current_h_left, current_x_left)
        rh_hip, rh_knee, rh_ankle = compute_human_ik(current_h_right, current_x_right)

        # APPLY URDF JOINT SIGNS from IK
        rh = rh_hip
        rk = -rh_knee
        ra = rh_ankle
        
        lh = -lh_hip
        lk = -lh_knee
        la = lh_ankle

        # ROLL CALCULATIONS (Parallelogram Fix)
        HIP_ROLL_GAIN = 1.0
        ANKLE_ROLL_GAIN = -1.0
        left_roll = -HIP_ROLL_GAIN * current_shift_left
        right_roll = -HIP_ROLL_GAIN * current_shift_right
        left_ankle_roll = ANKLE_ROLL_GAIN * current_shift_left
        right_ankle_roll = ANKLE_ROLL_GAIN * current_shift_right

        # Map to specific hardware IDs
        target_radians = {
            1:  current_arm_pitch_right, # Right Shoulder Pitch
            2:  current_arm_pitch_left,  # Left Shoulder Pitch
            3:  0.0,               # Right Shoulder Roll
            4:  0.0,               # Left Shoulder Roll
            5:  0.0,               # Right Elbow
            6:  0.0,               # Left Elbow
            
            7:  current_yaw_left,  # Left Yaw
            8:  current_yaw_right, # Right Yaw
            9:  right_roll,        # Right Hip Roll
            11: rh,                # Right Hip Pitch
            13: rk,                # Right Knee
            15: ra,                # Right Ankle Pitch
            17: right_ankle_roll,  # Right Ankle Roll
            
            10: left_roll,         # Left Hip Roll
            12: lh,                # Left Hip Pitch
            14: lk,                # Left Knee
            16: la,                # Left Ankle Pitch
            18: left_ankle_roll    # Left Ankle Roll
        }

        # Clear the syncwrite payload
        groupSyncWrite.clearParam()

        # Populate the bulk packet with translated ticks
        for motor_id, rad_angle in target_radians.items():
            tick = rad_to_tick(motor_id, rad_angle)
            param_goal_position = [DXL_LOBYTE(tick), DXL_HIBYTE(tick)]
            groupSyncWrite.addParam(motor_id, param_goal_position)

        # Blast all commands to the servos simultaneously
        dxl_comm_result = groupSyncWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print(f"SyncWrite Error: {packetHandler.getTxRxResult(dxl_comm_result)}")

        # Strict loop timing maintenance
        elapsed = time.perf_counter() - start_time
        if elapsed < loop_time:
            time.sleep(loop_time - elapsed)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[!] Ctrl+C Detected! Triggering Safe Shutdown...")
        abort_walk = True
        try:
            go_to_shutdown_pose()
        except:
            os._exit(0)
