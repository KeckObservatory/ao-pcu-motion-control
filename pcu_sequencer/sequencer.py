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
import os

# Static/global variables
TIME_DELAY = 0.5 # seconds
HOME = 0 # mm

# FIX Z-STAGE: Add to tolerance (copy m3)
TOLERANCE = {
    "m1": .01, # mm
    "m2": .008, # mm
    "m3": .005, # mm
}

MOVE_TIME = 45 # seconds
CLEARANCE_PMASK = 35 # mm, including mask radius
CLEARANCE_FIBER = 35 # mm, including fiber radius

# Undefined value for mini-move channels
RESET_VAL = -999.9 # mm, theoretically

### Logging
coloredlogs.DEFAULT_LOG_FORMAT = '%(asctime)s [%(levelname)s] %(message)s'
coloredlogs.DEFAULT_DATE_FORMAT = '%Y-%m-%d %H:%M:%S.%f'
coloredlogs.install(level='DEBUG')
log = logging.getLogger('')

# Config file and motor numbers
# FIX DEPLOY
config_file = "./PCU_configurations.yaml"
motor_file = "./valid_motors.yaml"

try:
    # Open and read config file with info on named positions
    with open(config_file) as f:
        file = f.read()
        config_lookup = yaml.load(file)
        # FIX-YAML version on k1aoserver-new is too old.
        #config_lookup = yaml.load(f, Loader=yaml.FullLoader)

    # Open and read motor file with info on valid motors
    with open(motor_file) as f: # FIX Z-STAGE
        file = f.read()
        valid_motors = yaml.load(file)['valid_motors']
except:
    print("Unable to read configuration files. Shutting down.")
    sys.exit(1)

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
        'enable_chan': ':enableRb',
        'spmg': '.SPMG',
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
    
    def isEnabled(self):
        """ Checks whether the motor is enabled """
        return not self.enable_chan.get()
    
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
        self.spmg.put('Stop')

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

    # FIX Z-STAGE
    home_Z = {'m3':0, 'm4':0}
    
    # -------------------------------------------------------------------------
    # Initialize the sequencer
    # -------------------------------------------------------------------------
    def __init__(self, prefix="k1:ao:pcu", tickrate=0.5):
        super().__init__(prefix, tickrate=tickrate)
        
        # Create new channel for metastate
        self._seqmetastate = self.ioc.registerString(f'{prefix}:meta')
        
        # Register individual motor channels
        for m_name in PCUSequencer.motors:
            chan_name = f"{m_name}Offset"
            # Register IOC channel for setting mini-moves
            setattr(self, "_"+chan_name, self.ioc.registerDouble(f'{prefix}:{chan_name}'))
            self.add_property(chan_name, dest_read=True)
            # Register IOC channel for readback
            setattr(self, "_"+chan_name+"Rb", self.ioc.registerDouble(f'{prefix}:{chan_name}Rb'))
            self.add_property(chan_name+"Rb")
        
        self.prepare(PCUStates)
        self.destination = None
        self.configuration = None
        self.motor_moves = []
        # Checks whether move has completed
        self.current_move = None
        
        # A timer for runtime usage
        self.move_timer = CountdownTimer()
    
    # -------------------------------------------------------------------------
    # Mini-move functions
    # -------------------------------------------------------------------------
    
    def add_property(self, p_name, dest_read=False):
        """ 
        Add a mini-move property to the PCUSequencer class
        named <p_name>, with or without a destructive read
        """
        chan_name = "_"+p_name
        
        # Define the getter channel for the property
        def getter(self):
            # Get the current value
            value = getattr(self, chan_name).get()
            
            if dest_read: # Clear value
                if value not in [RESET_VAL, None]:
                    getattr(self, chan_name).set(RESET_VAL)
                if value == RESET_VAL:
                    return None
            # Return set value
            return value
        
        # Set the property attributes
        setattr(PCUSequencer, p_name, # Set the self.<p_name> variable to the property described
                property( # property decorator
                    getter, # Getter method
                    lambda self, val: getattr(self, chan_name).set(val) # Setter method
                )
               )
    
    def get_mini_moves(self):
        """ Returns a dictionary of mini-moves to be taken """
        mini_moves = {}
        
        # Check all motor input channels
        for m_name in PCUSequencer.motors:
            offset_channel = m_name+"Offset"
            offset_request = getattr(self, offset_channel)
            # Check for requested moves
            if offset_request is not None:
                # Add to existing configuration
                if self.configuration in config_lookup:
                    offset_request += config_lookup[self.configuration][m_name]
                mini_moves[m_name] = offset_request
        
        return mini_moves
    
    def check_motor_limits(self, dest_pos):
        """ Get X and Y motor destinations and check limits """
        # I would like to add the limits to the YAML and streamline this
        
        if 'm1' in valid_motors:
            x_dest = dest_pos['m1']
            if x_dest > 305 or x_dest < 0:
                self.message("X-stage limit detected")
                return False
        
        if 'm2' in valid_motors:
            y_dest = dest_pos['m2']
            if y_dest > 202 or y_dest < 0:
                self.message("Y-stage limit detected.")
                return False
        
        if 'm3' in valid_motors:
            z1_dest = dest_pos['m3']
            if z1_dest > 100 or z1_dest < 0:
                self.message("Upper Z-stage limit detected.")
                return False
        
        if 'm4' in valid_motors:
            z2_dest = dest_pos['m4']
            if z2_dest > 100 or z2_dest < 0:
                self.message("Lower Z-stage limit detected.")
                return False
        
        return True
    
    def check_mini_moves(self, mini_moves):
        """ Checks that a move is valid within a configuration """
        # Check that it is the right configuration
        if self.configuration not in ['pinhole_mask', 'fiber_bundle']:
            return False
        
        # Get current motor positions
        dest_pos = self.get_positions()
        # Update to new positions
        for m_name, m_dest in mini_moves.items():
            dest_pos[m_name] = m_dest
        
        if not self.check_motor_limits(dest_pos):
            return False
        
        # Get centers of XY coordinates
        xc = config_lookup[self.configuration]['m1']
        yc = config_lookup[self.configuration]['m2']
        # Get move coordinates
        x_dest = dest_pos['m1']
        y_dest = dest_pos['m2']
        
        # Check for pinhole mask moves
        if self.configuration == "pinhole_mask":
            r_circ = CLEARANCE_PMASK # mm ### what do we want the clearance to be?
            # I'm going to need the exact center of the circle we want for this
            # The values we're using now are just estimates
            
            # OK to move pinhole mask, not fiber bundle
            # Maybe this should raise an error? Can you attach a string to a 
            #     low-level error and print out a warning higher up?
            if m_name == 'm3': return True
            if m_name == 'm4': return False
            
            # Check if XY motors are outside circle bounds
            return (x_dest-xc)**2 + (y_dest-yc)**2 < r_circ**2
        
        elif self.configuration == "fiber_bundle": # Check for fiber bundle moves
            r_circ = CLEARANCE_FIBER # mm
            # OK to move fiber bundle, not pinhole mask
            if m_name == 'm3': return False
            if m_name == 'm4': return True
            
            # Check if XY motors are outside circle bounds
            return (xc-x_dest)**2 + (yc-y_dest)**2 < r_circ
            
        else: # This shouldn't happen
            self.critical("Reached impossible state in checking mini-moves.")
            self.to_FAULT()
    
    # -------------------------------------------------------------------------
    # Regular motor-moving functions
    # -------------------------------------------------------------------------
    
    def get_positions(self):
        """ Returns positions of all valid motors """
        all_positions = {}
        for m_name, motor in PCUSequencer.motors.items():
            all_positions[m_name] = motor.get_pos()
        
        return all_positions
    
    def load_config(self, destination):
        """ Loads destination's moves into queue, clears current configuration """
        # Get ordered moves from destination state
        motor_posvals = config_lookup[destination]
        
        # Append info to move list
        self.motor_moves.clear()
        
        # Pull back Z stages if it's a major move
        if self.configuration != destination:
            self.motor_moves.append(PCUSequencer.home_Z)
        # Note: this should make it so moves within a configuration 
        #       don't pull the Z stages back all the way

        for m_name in motor_posvals.keys():
            # Skip bad entries in the yaml file.
            if m_name not in valid_motors:
                continue

            # Get destination of each motor
            dest = motor_posvals[m_name]

            # Append to motor moves
            self.motor_moves.append({m_name:dest})
        
        # Clear configuration and set destination
        self.configuration = None
        self.destination = destination

        return
    
    def trigger_move(self, m_dict):
        """ Triggers move and sets a timer to check if complete """
        for m_name, m_dest in m_dict.items():
            if m_name in valid_motors:
                # Get PV object for motor
                motor = PCUSequencer.motors[m_name]
                
                # Check that the motor is enabled
                if not motor.isEnabled():
                    self.critical(f"Motor {m_name} is not enabled.")
                    self.stop_motors()
                    self.to_FAULT()
                
                # Set position of motor
                motor.set_pos(m_dest)
        
        # Save current move to class variables
        self.current_move = m_dict
        
        # Start a timer for the move
        self.move_timer.start(seconds=MOVE_TIME)

        return
    
    def motor_in_position(self, m_name, m_dest):
        """ Checks whether a motor (m_name) is in position (m_dest) """
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
    
    def stop_motors(self):
        """ Stops motors only """
        
        # Message the thread
        self.critical("Stopping all motors.")
        
        # Clear future moves from queue
        self.current_move = None
        self.motor_moves.clear()
        # Set config to unknown
        self.configuration = None
        self.destination = None
        
        # Stop motors
        for _, pv in PCUSequencer.motors.items():
            pv.stop()
    
    def stop(self):
        """ Stops all PCU motors and halts operation """
        # Stop motors
        self.stop_motors()
        
        # Call the superclass stop method
        super().stop()
    
    # -------------------------------------------------------------------------
    # I/O processing
    # -------------------------------------------------------------------------
    
    def process_request(self):
        """ Processes input from the request keyword and returns it if valid """
        pass
    
    def checkabort(self):
        """Check if the abort flag is set, and drop into the FAULT state"""
        if self.seqabort:
            self.critical('Aborting sequencer!')
            self.stop()

        return False
    
    def checkmeta(self):
        if self.state == PCUStates.IN_POS:
            if self.configuration is None:
                self.metastate = "USER_DEF"
            else: self.metastate = self.configuration.upper()
        else:
            self.metastate = self.state.name
    
    def check_offsets(self):
        """ Checks offsets from the current configuration """
        if self.configuration not in config_lookup: # No metastate
            for m_name in PCUSequencer.motors:
                setattr(self, m_name+"OffsetRb", 0)
        else: # Get offset positions
            base_positions = config_lookup[self.configuration]
            motor_positions = self.get_positions()
            for m_name in motor_positions:
                offset = motor_positions[m_name] - base_positions[m_name]
                setattr(self, m_name+"OffsetRb", offset)
    
    # -------------------------------------------------------------------------
    # Init state
    # -------------------------------------------------------------------------
    
    def process_INIT(self):
        ###################################
        ## Any initialization stuff here ##
        ###################################
        
        
        ###################################
        self.to_IN_POS()
    
    # -------------------------------------------------------------------------
    # IN_POS state
    # -------------------------------------------------------------------------
    
    def process_IN_POS(self):
        """ Processes the IN_POS state """
        ######### Add mini-moves here ##########
        self.checkabort()
        self.checkmeta()
        self.check_offsets()
        
        try:
            # Check for mini-moves (dithers)
            mini_moves = self.get_mini_moves()
            # Found mini-moves
            if len(mini_moves) != 0:
                # Trigger a PCU move
                if self.check_mini_moves(mini_moves):
                    # Load mini-moves into queue
                    self.motor_moves.append(mini_moves)
                    # Need to set destination to preserve configuration
                    self.destination = self.configuration
                    # Go to moving
                    self.to_MOVING()
                else: # Warn user
                    self.critical(f"Invalid move for configuration {self.configuration}: {mini_moves}")
            
            # Check the request keyword
            request = self.seqrequest.lower()

            if request != '':
                # If move requested, process destination state
                if request.startswith('to_'):
                    destination = request[3:]
                    if destination in config_lookup:
                        self.message(f"Loading {destination} state.")
                        # Load next configuration (sets self.destination)
                        self.load_config(destination)
                        # Start move
                        self.to_MOVING()
                    else: self.critical(f'Invalid configuration: {destination}')

        # Enter the faulted state if a channel is disconnected while running
        except PVDisconnectException:
            # self.critical(message)
            self.stop_motors()
            self.to_FAULT()
    
    # -------------------------------------------------------------------------
    # MOVING state
    # -------------------------------------------------------------------------
    
    def process_MOVING(self):
        """ Process the MOVING state """
        self.checkabort()
        self.checkmeta()
        self.check_offsets()
        
        try:
            
            # Check for mini-move keywords
            mini_moves = self.get_mini_moves()

            # Check the request keyword and
            # start the reconfig process, if necessary
            request = self.seqrequest.lower()
            
            # Stop the PCU and go to USER_DEF position
            if request == 'stop':
                self.stop_motors()
                self.to_IN_POS()
            
            if request.startswith('to_') or len(mini_moves) != 0:
                self.critical("Send stop signal before moving to new position.")
            
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
                pass
            
            # Check if move has timed out
            if self.move_timer.expired:
                self.critical("Move failed due to motor timeout.")
                self.stop_motors()
                self.to_FAULT()

        # Enter the faulted state if a channel is disconnected while running
        except PVDisconnectException:
            self.stop_motors()
            self.to_FAULT()
    
    # -------------------------------------------------------------------------
    # FAULT state
    # -------------------------------------------------------------------------
    
    def process_FAULT(self):
        """ Processes the FAULT state """
        # Respond to request channel
        request = self.seqrequest.lower()
        if request == 'reinit':
            self.to_INIT()
    
    # -------------------------------------------------------------------------
    # TERMINATE state
    # -------------------------------------------------------------------------
    def process_TERMINATE(self):
        pass
    
    # -------------------------------------------------------------------------
    # Control channel properties (static)
    # -------------------------------------------------------------------------
    @property
    def metastate(self):
        request = self._seqmetastate.get()
        # Don't want a destructive read
        return request
    @metastate.setter
    def metastate(self, val): self._seqmetastate.set(val.encode('UTF-8'))

# -------------------------------------------------------------------------
# Main function
# -------------------------------------------------------------------------
if __name__ == "__main__":
    
    # Setup environment variables to find the right EPICS channel
    os.environ['EPICS_CA_ADDR_LIST'] = 'localhost:8600 localhost:8601 localhost:8602 ' + \
        'localhost:8603 localhost:8604 localhost:8605 localhost:8606 localhost:5064'
    os.environ['EPICS_CA_AUTO_ADDR_LIST'] = 'NO'

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
