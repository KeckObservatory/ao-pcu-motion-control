### pcu_sequencer.py : A document to contain the high-level sequencer code for all 5 named positions of the PCU
### Authors : Emily Ramey, Grace Jung
### Date : 11/22/21

### Imports
from transitions import Machine, State
from kPySequencer.Sequencer import Sequencer, PVDisconnectException, PVConnectException
import yaml
import numpy as np
import time
from epics import PV
from enum import Enum

TIME_DELAY = 0.5 # seconds
HOME = 0 # mm

# Config file and motor numbers
yaml_file = "/kroot/src/util/pcu_api/pcu_sequencer/PCU_configurations.yaml"
valid_motors = [f"m{i}" for i in np.arange(1,5)]

# Open and read config file with info on named positions
with open(yaml_file) as file:
    state_lookup = yaml.load(file, Loader=yaml.FullLoader)

# Getter and Setter channel names for each motor
set_pattern = "k1:ao:pcu:ln:{}:posval"
get_pattern = "k1:ao:pcu:ln:{}:posvalRb"

def move_motor(m_name, m_dest, block=True):
    print(f"Setting {m_name} to {m_dest} mm") # temporary
    
    # Get PVs for motor
    m_get = RunPCU.motors['get'][m_name]
    m_set = RunPCU.motors['set'][m_name]
    
    # Send move command
    m_set.put(m_dest)
    
    if block:
        # Block until motor is moved
        cur_pos = m_get.get()
        while cur_pos != m_dest: # Need a timeout and a tolerance or it may run forever
            # Get new position
            cur_pos = m_get.get()
            print(f"{m_name} position: {cur_pos}")
            # Wait for a short time
            time.sleep(TIME_DELAY)

class PCUStates(Enum):
    IN_POS = 0
    MOVING = 1
    FAULT = 2

# Class containing state machine
class PCUSequencer(Sequencer):
    
    def __init__(self, prefix, tickrate=0.5):
        super().__init__(prefix, tickrate)
        
        self.prepare(PCUStates)
    
    def process_IN_POS(self):
        try:
            # Wait for the user to set the desired request keyword and
            # start the reconfig process.
            request = self.seqrequest.lower()

            if request != '':
                if request == 'start':
                    self.message('Starting!')
                    self.to_MOVING()

        # Enter the faulted state if a channel is disconnected while running
        except PVDisconnectException:
            self.to_FAULT()

    def process_MOVING(self):
        try:
            # Wait for the user to set the desired request keyword and
            # start the reconfig process.
            request = self.seqrequest.lower()

            if request != '':
                if request == 'stop':
                    self.message('Stopping!')
                    self.to_IN_POS()

        # Enter the faulted state if a channel is disconnected while running
        except PVDisconnectException:
            self.to_FAULT()
    
    def process_FAULT(self):
        pass
    
    
#     # Initialize PCU states, on_enter moves motors into position
#     states = []
#     for name in state_lookup:
#         states.append(State(name=name, on_enter='move_motors'))
    
#     # Initialize epics PVs for motors
#     motors = {
#         'get': {},
#         'set': {}
#     }
#     # One getter and one setter PV per motor
#     for m_name in valid_motors:
#         motors['get'][m_name] = PV(get_pattern.format(m_name))
#         motors['set'][m_name] = PV(set_pattern.format(m_name))
    
#     # Initialize RunPCU instance
#     def __init__(self):
#         # Models 5 telescopes states, home Z stages before changing states
#         self.machine = Machine(model = self, states = RunPCU.states,
#                                before_state_change='home_Zstages',
#                                initial = 'telescope')
    
#     # Homes Z-stages before changing configuration
#     def home_Zstages(self):
#         move_motor('m3', HOME, block=False)
#         move_motor('m4', HOME, block=True)
    
#     # Moves stages/motors to new configuration
#     def move_motors(self):
#         # Get position values for new state
#         motor_posvals = state_lookup[self.state]
#         print(f"Moving motor to {motor_posvals} in state {self.state}")
        
#         # Move each motor, in order
#         for m_name in valid_motors:
#             # Get desired motor position in current state
#             m_dest = motor_posvals[m_name]
#             # Move motor
#             move_motor(m_name, m_dest)
            
            
