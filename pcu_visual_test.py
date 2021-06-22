# References http://spg.ucolick.org/KTLPython/index.html#

#! /usr/bin/env kpython

import ktl
import time
from . import pcu

def print_status():
    print("Reading the PCU's current status (field values).")
    pcu.curr_status()

def pcu_reset():
    # Move PCU back to default position ()
    print("Resetting PCU position.")
    pcu.reset()

def get_time():
    return time.time()
# --------------------------------------------------------
# Visual check of going from start -> specific position -> start -> etc.

# Check start up
print("Initiliazing the PCU controllers' channels.")
pcu.initialize_channels()
print_status()


# Pinhole Mask Position
print("Moving PCU to Pinhole Mask Position.")
pcu.pinhole_mask_position()
print_status()


# Fiber Bundle Position
pcu_reset()
print("Moving PCU to Fiber Bundle Position.")
pcu.fiber_bundle_position()
print_status()


# KPF Mirror Position
pcu_reset()
print("Moving PCU to KPF Mirror Position.")
pcu.KPF_mirror_position()
print_status()

# Telescope Sim Position
# As of now, I have this as the default PCU position (can be changed later)
pcu_reset()
print("Moving PCU to Telescope Sim Position.")
pcu.telescope_sim_position()
print_status()

# Telescope Position
pcu_reset()
print("Moving PCU to Telescope Position.")
pcu.telescope_position()
print_status()

# ---------------------------------------
# Time Test

# Testing time from sky position to KPF position
# Need to import time
pcu_reset()
pcu.telescope_position()
print_status()
t_start = get_time()
pcu.KPF_mirror_position()
t_end = get_time()
t = t_end - t_start
print_status()
print("")
print("")
print("Time from sky to KPF is ", t)


# ---------------------------------------
# Collision Avoidance
# These are the hard limits:
# X: -293mm to 49mm
# Y: -24.475mm to 206.525mm
