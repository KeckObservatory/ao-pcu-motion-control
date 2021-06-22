# References http://spg.ucolick.org/KTLPython/index.html#

#! /usr/bin/env kpython

import ktl

# Definitions (refers to naming of the service, keywords)

# Service name refers to Linear Stage Controller
SERVICE = 'LNR'
# Where Service object is stored
lnr = None

# Keywords follow the style: PCU:<DEVICE NAME>:<FIELD NAME>

# Heartbeat keywords for client-use to see if dispatchers working
# (Refers to each device with field values)
HEARTBEATS = ('DISPX1', 'DISPY1', 'DISPZ1', 'DISPZ2')

# Field values of linear stages (may not need explicit list, if using/populating all keywords by default)
FIELDS = []

# Keyword mapped to limits (may not be necessary because manually set up limits in hardware when setting up)
LIMITS = {}

"""
Initialize the Linear Controller as a Service. Monitor the controller's keywords.
"""
def start_up():
    # All keywords associated with Service are instantiated because populate=True
    lnr = ktl.Service(SERVICE, populate=True)

    # Making sure keyword regularly broadcasts to indicate Service is functional/connected
    for heartbeat in HEARTBEATS:
        # Default period is 5 seconds (can change)
        service.heartbeat(heartbeat)

    # Currently set to monitor ALL keywords (may change to subset after identifying ____ Linear stage keywords)
    lnr.monitor()

"""
Read the populated Keywords (instantiated Keywords) in the 'lnr' Service.
"""
def read_fields():
    # For each considered Keyword, read current field values
    for keyword in lnr.populated():
        print("Keyword ", keyword, ": ", keyword.read())

"""
Set component in the ____ Linear Stage, to the desired raw, user, or dial coordinates position.
Value: Positive argument means _________
Units:
"""
def move_X_stage_to(coord):
    lnr['RVAL'].write(coord)

def move_Y_stage_to(coord):
    lnr['RVAL'].write(coord)

def move_Z1_stage_to(coord):
    lnr['RVAL'].write(coord)

def move_Z2_stage_to(coord):
    lnr['RVAL'].write(coord)

"""
Change the field of a Keyword
Name: Field name for Keyword
Value: New value to write to field value
"""
def update_field(name, value):
    keyword = lnr[name]
    # Should probably do some error checking here
    if !keyword['populated']:
        return
    keyword.write(value)


# -----------------------------------------------------------------------------
# Ignore just for now (I think safer to have default position an actual measurement-defined position)
"""
Move stage to default position.
"""
def default_position():
