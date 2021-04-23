# References http://spg.ucolick.org/KTLPython/index.html#

#! /usr/bin/env kpython

import ktl
import pcu_linear_controller as lnr
import pcu_rotator_controller as rtr

initialized_ktl = False

# Devices:
# --- ARE DEVICES CONTROLLERS and STAGES? Or just controllers?
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

# -----------------------------------------------------------
# Below code follows if PCU controllers are the service versus the PCU itself being it
def initialize_channels():
    lnr.start_up()
    rtr.start_up()
    lnr.read_fields()
    rtr.read_fields()
    initialized_ktl = True

def reset():
    lnr.default_position()
    rtr.default_position()

# Specific PCU Positions
def pinhole_mask_position():
    # Move X Stage:
    raise NotImplementedError

def fiber_bundle_position():
    raise NotImplementedError

def KPF_mirror_position():
    raise NotImplementedError

def telescope_sim_position():
    raise NotImplementedError

def telescope_position():
    raise NotImplementedError
