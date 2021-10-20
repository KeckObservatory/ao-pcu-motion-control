### pcu_sequencer.py : A document to contain the high-level sequencer code for all 5 named positions of the PCU
### Authors : Emily Ramey, Grace Jung
### Date : 11/22/21

### Imports
from transitions import Machine, State
from kPySequencer.Sequencer import Sequencer, PVDisconnectException, PVConnectException
from kPySequencer.Tasks import Tasks
from kPySequencer.CountdownTimer import CountdownTimer
import logging, coloredlogs
import yaml
import numpy as np
import time
from epics import PV
from enum import Enum
import signal
import sys

# Static/global variables
TIME_DELAY = 0.5 # seconds
HOME = 0 # mm
TOLERANCE = {
    "m1": .01, # mm
    "m2": .008, # mm
    "m3": .005, # mm
}
MOVE_TIME = 90 # seconds
CLEARANCE_PMASK = 26.5 # mm, including mask radius
CLEARANCE_FIBER = 23.5 # mm, including fiber bundle

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

# Motor class
class PCUMotor():
    
    # Patterns for motor functions
    base_pattern = "k1:ao:pcu"
    channels = {
        'get_chan': ":posvalRb",
        'set_chan': ":posval",
        'halt_chan': ":halt",
        'jog_chan': ':jog',
        'go_chan': ':go',
        'spmg': '.SPMG'
    }
    
    def __init__(self, m_name, m_type="ln"):
        
        self.channel_list = []
        
        # Set up all channel PVs as attributes
        for channel_key, channel_pat in PCUMotor.channels.items():
            # Assemble full channel name
            full_channel = f"{PCUMotor.base_pattern}:{m_type}:{m_name}{channel_pat}"
            # Create EPICS PV
            channel_PV = PV(full_channel)
            self.channel_list.append(channel_PV)
            # Set attribute
            setattr(self, channel_key, channel_PV)
            # Set attribute name
            setattr(self, channel_key+"_name", full_channel)
    
    def check_connection(self):
        for pv in self.channel_list:
            if not pv.connect():
                print(f"Channel {pv.pvname} has disconnected.")
                raise PVDisconnectException
    
    def get_pos(self):
        self.check_connection()
        return self.get_chan.get()

    def set_pos(self, pos):
        self.check_connection()
        self.set_chan.put(pos)
        self.go_chan.put(1)

    def stop(self): 
        # Important that this doesn't check connection,
        # as a stop can result from a disconnect exception
        # Should that be the case?
        self.spmg.put('Stop')
    

# Getter and Setter channel names for each motor
set_pattern = "k1:ao:pcu:ln:{}:posval"
get_pattern = "k1:ao:pcu:ln:{}:posvalRb"
halt_pattern = "k1:ao:pcu:ln:{}:halt"

class PCUStates(Enum):
    INIT = 0
    IN_POS = 1
    MOVING = 2
    FAULT = 3
    TERMINATE = 4

# Class containing state machine
class PCUSequencer(Sequencer):

    # Initialize epics PVs for motors
    motors = {
        m_name: PCUMotor(m_name) for m_name in valid_motors
    }
    
    home_Z = {'m3':0, 'm4':0}
    
    def __init__(self, prefix="k1:ao:pcu", tickrate=0.5):
        super().__init__(prefix, tickrate=tickrate)
        
        # Create new channel for metastate
        self._seqmetastate = self.ioc.registerString(f'{prefix}:meta')
        
        # Register motor channels
        for m_name in PCUSequencer.motors:
            chan_name = f"{m_name}_rel"
            # Register IOC channel
            setattr(self, "_"+chan_name, self.ioc.registerLong(f'{prefix}:{chan_name}'))
        
        self.prepare(PCUStates)
        self.destination = None
        self.configuration = None
        self.motor_moves = []
        # Checks whether move has completed
        self.current_move = None
        
        # A timer for runtime usage
        self.move_timer = CountdownTimer()
        
        self.message(self.m1_rel.get())
    
    def motor_in_position(self, m_name, m_dest):
        # Get PV getter for motor
        motor = PCUSequencer.motors[m_name]
        # Get current position
        cur_pos = motor.get_pos()
        
        # Compare to destination within tolerance, return False if not reached
        t = TOLERANCE[m_name]
        # Lower and upper limits
        in_pos = cur_pos > m_dest-t and cur_pos < m_dest+t
        # Return whether the given motor is in position
        return in_pos
    
    def load_config(self):
        """ Loads a configuration into class variables """
        # Get ordered moves from destination state
        motor_posvals = config_lookup[self.destination]
        
        # Append info to move list
        self.motor_moves.clear()
        self.motor_moves.append(PCUSequencer.home_Z)

        for m_name in valid_motors:
            # Get destination of each motor
            dest = motor_posvals[m_name]
            # Append to motor moves
            self.motor_moves.append({m_name:dest})
    
    def get_positions():
        """ Returns positions of all valid motors """
        all_positions = {}
        for m_name, motor in PCUSequencer.motors:
            all_positions[m_name] = motor.get_pos()
        
        return all_positions
    
    def check_move(self, m_name, m_dest):
        """ Checks that a move is valid within a configuration """
        # Get current motor positions
        dest_pos = self.get_positions()
        dest_pos[m_name] = m_dest
        # Get X and Y motor destinations
        x_dest = dest_pos['m1']
        y_dest = dest_pos['m2']
        
        # Get centers of XY coordinates
        xc = config_lookup[self.configuration]['m1']
        yc = config_lookup[self.configuration]['m2']
        
        # Check for pinhole mask moves
        if self.configuration == "pinhole_mask":
            r_circ = 42 # mm ### what do we want the clearance to be?
            # I'm going to need the exact center of the circle we want for this
            # The values we're using now are just estimates
            
            # OK to move pinhole mask, not fiber bundle
            # Maybe this should raise an error? Can you attach a string to a 
            #     low-level error and print out a warning higher up?
            if m_name == 'm3': return True
            if m_name == 'm4': return False
            
            # Check if XY motors are outside circle bounds
            return (xc-x_dest)**2 + (yc-y_dest)**2 < r_circ**2
        
        # Check for fiber bundle moves
        if self.configuration == "fiber_bundle":
            r_circ = 45 # mm ### no idea on this one
            # OK to move fiber bundle, not pinhole mask
            if m_name == 'm3': return False
            if m_name == 'm4': return True
            
            # Check if XY motors are outside circle bounds
            return (xc-x_dest)**2 + (yc-y_dest)**2 < r_circ
        
        else: # Can't move in any other configuration atm
            return False
    
    def trigger_move(self, m_dict):
        """ Triggers move and sets a callback to check if complete """
        for m_name, m_dest in m_dict.items():
            if m_name in valid_motors:
                self.check_move(m_name, m_dest)
                # Get PV setter for motor
                m_set = PCUSequencer.motors[m_name]
                # Set position
                m_set.set_pos(m_dest)
        # Save current move to class variables
        self.current_move = m_dict
        
        # Start a timer for the move
        self.move_timer.start(seconds=MOVE_TIME)
    
    def move_complete(self):
        """ Returns True when the move in self.current_move is complete """
        # Get current motor motions
        m_dict = self.current_move
        # Return True if no moves are taking place
        if m_dict is None: return True
        
        # Get current positions and compare to destinations
        for m_name, m_dest in m_dict.items():
            if m_name in valid_motors:
                if not self.motor_in_position(m_name, m_dest):
                    return False
                
        # Return True if motors are in position and release current_move
        self.message(f"Move {self.current_move} complete!")
        self.current_move = None
        return True
    
    def checkmeta(self):
        if self.state == PCUStates.IN_POS:
            if self.configuration is None:
                self.metastate = "USER_DEF"
            else: self.metastate = self.configuration.upper()
        else:
            self.metastate = self.state.name
    
    def process_INIT(self):
        ###################################
        ## Any initialization stuff here ##
        ###################################
        
        
        ###################################
        self.to_IN_POS()
    
    def process_IN_POS(self):
        """ Processes the IN_POS state """
        ######### Add mini-moves here ##########
        self.checkabort()
        self.checkmeta()
        self.m1_rel.set(10)
        try:
            # Wait for the user to set the desired request keyword and
            # start the reconfig process.
            request = self.seqrequest.lower()

            if request != '':
                self.message(request)
                # Process destination
                if request.startswith('to_'):
                    destination = request[3:]
                    if destination in config_lookup:
                        self.message(f"Loading {destination} state.")
                        # Clear configuration and set destination
                        self.configuration = None
                        self.destination = destination
                        # Load next configuration
                        self.load_config()
                        # Start move
                        self.to_MOVING()
                    else: self.critical(f'Invalid configuration: {destination}')

        # Enter the faulted state if a channel is disconnected while running
        except PVDisconnectException:
            # self.critical(message)
            self.stop_motors()
            self.to_FAULT()

    def process_MOVING(self):
        """ Process the MOVING state """
        self.checkabort()
        self.checkmeta()
        try:
            # Check the request keyword and
            # start the reconfig process, if necessary
            request = self.seqrequest.lower()
            
            # If there are moves in the queue and previous moves are done
            if len(self.motor_moves) != 0 and self.move_complete():
                # There are moves in the queue, pop next move from the list and trigger it
                next_move = self.motor_moves.pop(0)
                self.message(f"Triggering move, {next_move}.")
                self.trigger_move(next_move)
            elif len(self.motor_moves) == 0 and self.move_complete():
                # No moves left to make, finish and change state
                self.message("Finished moving.")
                # Change configuration and destination keywords
                self.configuration = self.destination
                self.destination = None
                # Move to in-position state
                self.to_IN_POS()
            else: # Move is in progress
                self.message(self.move_timer.elapsed)
            
            # Check if move has timed out
            if self.move_timer.expired:
                self.critical("Move failed.")
                self.stop_motors()
                self.to_FAULT()

        # Enter the faulted state if a channel is disconnected while running
        except PVDisconnectException:
            self.stop_motors()
            self.to_FAULT()
    
    def process_FAULT(self):
#         self.stop_motors()
        
        # Respond to request channel
        request = self.seqrequest.lower()
        if request == 'reinit':
            self.to_INIT()
    
    def process_TERMINATE(self):
        pass
    
    def stop_motors(self):
        """ Stop motors. """
        self.motor_moves.clear()
        for _, pv in PCUSequencer.motors.items():
            pv.stop()
    
    def stop(self):
        """ Stop all the motors and halt operation. """
        # Message the thread
        self.critical("Stopping all motors.")
        
        # Stop motors
        self.stop_motors()
        
        # Call the superclass stop method
        super().stop()
    
    # -------------------------------------------------------------------------
    def checkabort(self):
        """Check if the abort flag is set, and drop into the FAULT state"""
        if self.seqabort:
            self.critical('Aborting sequencer!')
            self.stop_motors()
            self.to_FAULT()
            return True

        return False
    
    # Wrap the control channels in properties so they can be accessed like variables
    @property
    def metastate(self):
        # Clear the request channel automatically!
        request = self._seqmetastate.get()
        # Don't want a destructive read
        return request
    @metastate.setter
    def metastate(self, val): self._seqmetastate.set(val.encode('UTF-8'))
    
    # Set up motor variables for easier getting / setting
    # Note: Make these safe for disabled motors
    @property
    def m1_rel(self):
        return self._m1_rel.get()
    @m1_rel.setter
    def m1_rel(self, val): self._m1_rel.set(val)
    
    @property
    def m2_rel(self):
        return self._m2_rel.get()
    @m2_rel.setter
    def m2_rel(self, val): self._m2_rel.set(val)
    
    @property
    def m3_rel(self):
        return self._m3_rel.get()
    @m2_rel.setter
    def m3_rel(self, val): self._m3_rel.set(val)

if __name__ == "__main__":

    # Define an enum of task names
    class TASKS(Enum):
        SequencerTask1 = 0

    # The main sequencer
    setup = PCUSequencer(prefix='k1:ao:pcu')

    # Create a task pool and register the sequencers that need to run
    tasks = Tasks(TASKS, 'k1:ao:pcu', workers=len(TASKS))
    tasks.register(setup, TASKS.SequencerTask1)

    # Start everything
    log.info('Starting sequencer.')
    tasks.run()