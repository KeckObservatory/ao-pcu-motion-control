# References http://spg.ucolick.org/KTLPython/index.html#

#! /usr/bin/env kpython

import ktl
from . import pcu_linear_controller as lnr
from . import pcu_rotator_controller as rtr

initialized_ktl = False

# Devices:
# --- ARE DEVICES CONTROLLERS and STAGES? Or just controllers? (when using the kpython/EPICS syntax)
controllers = (lnr, rtr)
# stages = (x_stage, y_stage, z1_stage, z2_stage)

# I want check understanding of connecting PI and Keck document layout
def start_pcu_service():
    # Populate all PCU keywords under PCU service
    pcu_service = ktl.Service("PCU", populate=True)
    # Monitor all keywords
    pcu_service.monitor()
    for keyword in pcu_service.populated():
        print("Keyword ", keyword, ": ", keyword.read())
    initialized_ktl = True

# Above is if PCU itself is it's own service
# -----------------------------------------------------------
# Below code follows if PCU controllers are the service versus the PCU itself being it
def curr_status():
    lnr.read_fields()
    rtr.read_fields()

def initialize_channels():
    lnr.start_up()
    rtr.start_up()
    initialized_ktl = True

# Specific PCU Positions (as defined by the design document)
def pinhole_mask_position():
    # Move X Stage to -173.375 mm
    lnr.move_X_stage_to(-173.375)
    # Move Y Stage to 69 mm
    lnr.move_Y_stage_to(0)
    # Rotate
    rtr.rotate_raw_coord(55)

def fiber_bundle_position(69):
    # Move X Stage to  mm
    lnr.move_X_stage_to(-173.375)
    # Move Y Stage to  mm
    lnr.move_Y_stage_to(69)
    # Rotate
    rtr.rotate_raw_coord(0)

def KPF_mirror_position():
    # Move X Stage to  mm
    lnr.move_X_stage_to(-193.706)
    # Move Y Stage to  mm
    lnr.move_Y_stage_to(140)
    # Rotate
    rtr.rotate_raw_coord(50)

def telescope_sim_position():
    # Move X Stage to  mm
    lnr.move_X_stage_to(0)
    # Move Y Stage to  mm
    lnr.move_Y_stage_to(50)
    # Rotate
    rtr.rotate_raw_coord(50)

def telescope_position():
    # Move X Stage to  mm
    lnr.move_X_stage_to(-276)
    # Move Y Stage to  mm
    lnr.move_Y_stage_to(140)
    # Rotate
    rtr.rotate_raw_coord(109)

def reset():
    #Using telescope_sim_position since appears to be 0 location on X stage
    telescope_sim_position()
    #Before, if considering some position other than design pcu positions
    #lnr.default_position()
    #rtr.default_position()
