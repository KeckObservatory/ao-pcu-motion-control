# References http://spg.ucolick.org/KTLPython/index.html#

#! /usr/bin/env kpython

import ktl
import pcu_linear_controller as lnr
import pcu_rotator_controller as rtr

initialized_ktl = False

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
    raise NotImplementedError

def fiber_bundle_position():
    raise NotImplementedError

def KPF_mirror_position():
    raise NotImplementedError

def telescope_sim_position():
    raise NotImplementedError

def telescope_position():
    raise NotImplementedError
