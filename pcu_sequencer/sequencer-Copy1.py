### pcu_sequencer.py : A document to contain the high-level sequencer code for all 5 named positions of the PCU
### Authors : Emily Ramey, Grace Jung
### Date : 11/22/21

### Imports
from transitions import Machine, State
from kPySequencer.Sequencer import Sequencer, PVDisconnectException, PVConnectException
from kPySequencer.Tasks import Tasks
import logging, coloredlogs
import yaml
import numpy as np
import time
from epics import PV
from enum import Enum

TIME_DELAY = 0.5 # seconds
HOME = 0 # mm

### Logging
coloredlogs.DEFAULT_LOG_FORMAT = '%(asctime)s [%(levelname)s] %(message)s'
coloredlogs.DEFAULT_DATE_FORMAT = '%Y-%m-%d %H:%M:%S.%f'
coloredlogs.install(level='DEBUG')
log = logging.getLogger('')

# Config file and motor numbers
yaml_file = "/kroot/src/util/pcu_api/pcu_sequencer/PCU_configurations.yaml"
# valid_motors = [f"m{i}" for i in np.arange(1,5)]
valid_motors = [f"m{i}" for i in np.arange(1, 4)] # If fiber bundle motor isn't working

# Open and read config file with info on named positions
with open(yaml_file) as file:
    config_lookup = yaml.load(file, Loader=yaml.FullLoader)

# Getter and Setter channel names for each motor
set_pattern = "k1:ao:pcu:ln:{}:posval"
get_pattern = "k1:ao:pcu:ln:{}:posvalRb"

# def move_motor(m_name, m_dest, block=True):
#     print(f"Setting {m_name} to {m_dest} mm") # temporary
    
#     # Get PVs for motor
#     m_get = PCUSequencer.motors['get'][m_name]
#     m_set = PCUSequencer.motors['set'][m_name]
    
#     # Send move command
#     m_set.put(m_dest)
    
#     if block:
#         # Block until motor is moved
#         cur_pos = m_get.get()
#         while cur_pos != m_dest: # Need a timeout and a tolerance or it may run forever
#             # Get new position
#             cur_pos = m_get.get()
#             print(f"{m_name} position: {cur_pos}")
#             # Wait for a short time
#             time.sleep(TIME_DELAY)

class PCUStates(Enum):
    IN_POS = 0
    MOVING = 1
    FAULT = 2

# Class containing state machine
class PCUSequencer(Sequencer):

    # Initialize epics PVs for motors
    motors = { # One getter and one setter PV per motor
        'get': {m_name: PV(get_pattern.format(m_name)) for m_name in valid_motors},
        'set': {m_name: PV(set_pattern.format(m_name)) for m_name in valid_motors}
    }
    
    home_Z = {'m3':0, 'm4':0}
    
    def __init__(self, prefix, tickrate=0.5):
        super().__init__(prefix, tickrate=tickrate)
        
        self.prepare(PCUStates)
        self.destination = None
        self.motor_moves = []
        # Checks whether move has completed
        self.current_move = None
    
    def load_config(self, destination):
        """ Loads a configuration into class variables """
        # Append info to move list
        self.motor_moves.clear()
        self.motor_moves.append(PCUSequencer.home_Z)

        # Get ordered moves from destination state
        motor_posvals = config_lookup[destination]
        for m_name in valid_motors:
            # Get destination of each motor
            dest = motor_posvals[m_name]
            # Append to motor moves
            self.motor_moves.append({m_name:dest})
    
    def trigger_move(self, m_dict):
        """ Triggers move and sets a callback to check if complete """
        for m_name, m_dest in m_dict.items():
            if m_name in valid_motors:
                # Get PV setter for motor
                m_set = PCUSequencer.motors['set'][m_name]
                # Set position
                m_set.put(m_dest)
        # Save current move to class variables
        self.current_move = m_dict
    
    def move_complete(self):
        """ Returns True when the move in self.current_move is complete """
        # Get current motor motions
        m_dict = self.current_move
        # Return True if no moves are taking place
        if m_dict is None: return True
        
        # Get current positions and compare to destinations
        for m_name, m_dest in m_dict.items():
            if m_name in valid_motors:
                # Get PV getter for motor
                m_get = PCUSequencer.motors['get'][m_name]
                # Get current position
                cur_pos = m_get.get()
                # Compare to destination, return False if not reached
                if cur_pos != m_dest:
                    return False
        # Return True if motors are in position and release current_move
        self.message(f"{self.current_move} complete!")
        self.current_move = None
        return True
    
    def process_IN_POS(self):
        """ Processes the IN_POS state """
        try:
            # Wait for the user to set the desired request keyword and
            # start the reconfig process.
            request = self.seqrequest.lower()

            if request != '':
                # Process destination
                if request.startswith('to_'):
                    destination = request[3:]
                    if destination in config_lookup:
                        self.message(f"Loading {destination} state.")
                        self.destination = destination
                        self.load_config(self.destination)
                        self.to_MOVING()
                    else: self.message(f'Invalid configuration: {destination}')
                
                elif request == 'abort':
                    self.to_FAULT()

        # Enter the faulted state if a channel is disconnected while running
        except PVDisconnectException:
            self.to_FAULT()

    def process_MOVING(self):
        try:
            # Check the request keyword and
            # start the reconfig process, if necessary
            request = self.seqrequest.lower()
            
            if request != '':
                if request == 'stop':
                    self.message('Stopping!')
                    self.to_IN_POS() # Should it go to fault?
            
            # If there are moves left and previous moves are done, do the next move
            if len(self.motor_moves) != 0 and self.move_complete():
                # Pop from the list and trigger
                next_move = self.motor_moves.pop(0)
                self.message(f"Triggering move, {next_move}.")
                self.trigger_move(next_move)
            elif len(self.motor_moves) == 0 and self.move_complete(): # No moves left
                self.message("Finished moving.")
                self.to_IN_POS()
#             else: self.message(f"Moving, {self.current_move}")
            
            self.message(f"Moves: {self.motor_moves}, Current: {self.current_move}, Complete: {self.move_complete()}")

        # Enter the faulted state if a channel is disconnected while running
        except PVDisconnectException:
            self.to_FAULT()
    
    def process_FAULT(self):
        pass
    
#     # Initialize PCU states, on_enter moves motors into position
#     states = []
#     for name in config_lookup:
#         states.append(State(name=name, on_enter='move_motors'))
    
#     # Initialize PCUSequencer instance
#     def __init__(self):
#         # Models 5 telescopes states, home Z stages before changing states
#         self.machine = Machine(model = self, states = PCUSequencer.states,
#                                before_state_change='home_Zstages',
#                                initial = 'telescope')
    
#     # Homes Z-stages before changing configuration
#     def home_Zstages(self):
#         move_motor('m3', HOME, block=False)
#         move_motor('m4', HOME, block=True)
    
#     # Moves stages/motors to new configuration
#     def move_motors(self):
#         # Get position values for new state
#         motor_posvals = config_lookup[self.state]
#         print(f"Moving motor to {motor_posvals} in state {self.state}")
        
#         # Move each motor, in order
#         for m_name in valid_motors:
#             # Get desired motor position in current state
#             m_dest = motor_posvals[m_name]
#             # Move motor
#             move_motor(m_name, m_dest)
            

if __name__ == "__main__":

    # Define an enum of task names
    class TASKS(Enum):
        SequencerTask1 = 0

    # The main sequencer
    setup = PCUSequencer(prefix='test')

    # Create a task pool and register the sequencers that need to run
    tasks = Tasks(TASKS, 'test', workers=len(TASKS))
    tasks.register(setup, TASKS.SequencerTask1)

    # Start everything
    log.info('Starting sequencer.')
    tasks.run()