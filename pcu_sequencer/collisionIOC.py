### collisionIOC.py : A document to monitor the individual PCU motor channels and stop them if there is an imminent collision
### Authors : Emily Ramey
### Date : 12/8/21

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

import PCU_util as util
from positions import PCUPos, PCUMove
from motors import PCUMotor

### Logging
coloredlogs.DEFAULT_LOG_FORMAT = '%(asctime)s [%(levelname)s] %(message)s'
coloredlogs.DEFAULT_DATE_FORMAT = '%Y-%m-%d %H:%M:%S.%f'
coloredlogs.install(level='DEBUG')
log = logging.getLogger('')

class collisionStates(Enum):
    INIT = 0
    STANDBY = 1
    STOPPED = 2
    RESTRICTED = 3
    TERMINATE = 4

# Class containing state machine
class collisionSequencer(Sequencer):
    
    # -------------------------------------------------------------------------
    # Initialize the sequencer
    # -------------------------------------------------------------------------
    def __init__(self, prefix="collisions", tickrate=0.5):
        super().__init__(prefix, tickrate=tickrate)
        
        # Create new channel for metastate
        self._seqmetastate = self.ioc.registerString(f'{prefix}:stst')
        
        self.prepare(collisionStates)
    
    def load_config_files(self):
        """ Loads configuration files into class variables """
        # Update position class
        PCUPos.load_motors()
        
        # Load configuration files
        all_configs = util.load_configurations()
        self.base_configs = PCUPos.from_dict(all_configs[0])
        self.fiber_configs = PCUPos.from_dict(all_configs[1])
        self.mask_configs = PCUPos.from_dict(all_configs[2])
        
        # Load motor configuration info
        motor_info = util.load_motors()
        
        # Assign motor info to variables
        self.valid_motors = motor_info['valid_motors']
        self.motor_limits = motor_info['motor_limits']
        self.tolerance = motor_info['tolerance']

        # Assign config info to variables
        self.all_configs = dict(self.base_configs, **self.fiber_configs, **self.mask_configs)
        self.user_configs = dict(self.fiber_configs, **self.mask_configs)
    
    def load_motors(self, prefix):
        """ Loads valid motors into class variable """
        # Initialize epics PVs for motors
        self.motors = {
            m_name: PCUMotor(m_name) for m_name in self.valid_motors
        }
    
    def stop(self):
        """ halts operation """
        # Call the superclass stop method
        super().stop()
    
    def current_pos(self):
        """ Returns positions of all valid motors """
        cur_pos = PCUPos()
        for m_name, motor in self.motors.items():
            cur_pos[m_name] = motor.get_pos()
        
        return cur_pos
    
    def commanded_pos(self):
        """ Return commanded position of all valid motors """
        pos = PCUPos()
        for m_name, motor in self.motors.items():
            pos[m_name] = motor.get_commanded()
            
        return pos
    
    def stop_motors(self):
        """ Stops motors only """
        
        # Message the thread
        self.critical("Stopping all motors.")
        
        # Stop motors
        for _, pv in self.motors.items():
            pv.stop()
            pv.disable()
    
    # -------------------------------------------------------------------------
    # I/O processing
    # -------------------------------------------------------------------------
    
    def process_request(self):
        """ Processes input from the request keyword """
        pass
    
    def checkabort(self):
        """ Check if the abort flag is set, and stop the sequencer if so """
        if self.seqabort:
            self.critical('Aborting sequencer!')
            self.stop_motors()
            self.stop()

        return False
    
    def checkmeta(self):
        """ Checks the metastate """
        self.metastate = self.state.name
    
    # -------------------------------------------------------------------------
    # Init state
    # -------------------------------------------------------------------------
    
    def process_INIT(self):
        self.checkmeta()
        try:
            ###################################
            ## Any initialization stuff here ##
            ###################################

            ###################################
            PCUPos.load_motors()
            self.to_MONITORING()
        except PVDisconnectException as err:
            self.critical(str(err))
            self.stop_motors()
            self.to_STOPPED()
    
    # -------------------------------------------------------------------------
    # MONITORING state
    # -------------------------------------------------------------------------
    def process_MONITORING(self):
        """ Check current position and stop if it's not valid """
        self.checkabort()
        self.checkmeta()
        self.process_request()
        try:
            cur_pos = self.current_pos()
            future_pos = self.commanded_pos()
            
            if not cur_pos.is_valid():
                self.critical(f"Current position is invalid: {cur_pos}. Disabling all motors.")
                self.stop_motors()
                self.to_STOPPED()
            elif not future_pos.is_valid():
                self.critical(f"Commanded position is invalid {future_pos}. Disabling all motors")
                self.stop_motors()
                self.to_STOPPED()
        # Enter the STOPPED state if a channel is disconnected while running
        except PVDisconnectException as err:
            self.critical(str(err))
            self.stop_motors()
            self.to_STOPPED()
    
    # -------------------------------------------------------------------------
    # STOPPED state
    # -------------------------------------------------------------------------
    def process_STOPPED(self):
        # Send stop command until it's valid again?
        self.checkabort()
        self.checkmeta()
        
        # Make sure the motors are disabled
        self.stop_motors()
    
    # -------------------------------------------------------------------------
    # RESTRICTED state
    # -------------------------------------------------------------------------
    def process_RESTRICTED(self):
        # Send stop command until it's valid again?
        self.checkabort()
        self.checkmeta()
        
        # Check which axis/axes should allow motion
        
        
        # Continually disable all other motors
    
    # -------------------------------------------------------------------------
    # TERMINATE state
    # -------------------------------------------------------------------------
    def process_TERMINATE(self):
        # Send stop command until it's valid again?
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
#     os.environ['EPICS_CA_ADDR_LIST'] = 'localhost:8600 localhost:8601 localhost:8602 ' + \
#         'localhost:8603 localhost:8604 localhost:8605 localhost:8606 localhost:5064'
#     os.environ['EPICS_CA_AUTO_ADDR_LIST'] = 'NO'

    # Define an enum of task names
    class TASKS(Enum):
        SequencerTask1 = 0

    # The main sequencer
    setup = PCUSequencer(prefix='k1:ao:pcu')

    # Create a task pool and register the sequencers that need to run
    tasks = Tasks(TASKS, 'collisions', workers=len(TASKS))
    tasks.register(setup, TASKS.SequencerTask1)

    # Start everything
    log.info('Starting collision sequencer.')
    tasks.run()