### testIOC.py : A document to set up and run several test IOC channels
### Authors : Emily Ramey
### Date : 11/29/21

# import argparse
# parser = argparse.ArgumentParser()
# parser.add_argument("--port", help="Port for the sequencer to run on")
# args = parser.parse_args()

import logging, coloredlogs
import os

port = 6200
log.info(f'Setting server port to {port}')
os.environ['EPICS_CA_SERVER_PORT'] = port

### Imports
from transitions import Machine, State
from kPySequencer.Sequencer import Sequencer, PVDisconnectException, PVConnectException
from kPySequencer.Tasks import Tasks
from kPySequencer.CountdownTimer import CountdownTimer

import yaml
import numpy as np
import time
from epics import PV
from enum import Enum
import signal
import sys

### Logging
coloredlogs.DEFAULT_LOG_FORMAT = '%(asctime)s [%(levelname)s] %(message)s'
coloredlogs.DEFAULT_DATE_FORMAT = '%Y-%m-%d %H:%M:%S.%f'
coloredlogs.install(level='DEBUG')
log = logging.getLogger('')

# Config file and motor numbers
# FIX DEPLOY
# config_file = "./PCU_configurations.yaml"
# motor_file = "./valid_motors.yaml"

class testStates(Enum):
    INIT = 0
    STATIC = 1
    TERMINATE = 2

# Class containing state machine
class testSequencer(Sequencer):
    
    # -------------------------------------------------------------------------
    # Initialize the sequencer
    # -------------------------------------------------------------------------
    def __init__(self, prefix="test", tickrate=0.5):
        super().__init__(prefix, tickrate=tickrate)
        
        # Create new channel for metastate
        self._seqmetastate = self.ioc.registerString(f'{prefix}:meta')
        self._testDouble = self.ioc.registerDouble(f'{prefix}:test_double')
        
        self.prepare(testStates)
    
    def stop(self):
        """ halts operation """
        # Call the superclass stop method
        super().stop()
    
    # -------------------------------------------------------------------------
    # I/O processing
    # -------------------------------------------------------------------------
    
    def process_request(self):
        """ Processes input from the request keyword """
        pass
    
    def checkabort(self):
        """Check if the abort flag is set, and drop into the FAULT state"""
        if self.seqabort:
            self.critical('Aborting sequencer!')
            self.stop()

        return False
    
    # -------------------------------------------------------------------------
    # Init state
    # -------------------------------------------------------------------------
    
    def process_INIT(self):
        ###################################
        ## Any initialization stuff here ##
        ###################################
        
        ###################################
        self.to_STATIC()
    
    # -------------------------------------------------------------------------
    # STATIC state
    # -------------------------------------------------------------------------
    def process_STATIC(self):
        pass
    
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
    setup = testSequencer(prefix='test')

    # Create a task pool and register the sequencers that need to run
    tasks = Tasks(TASKS, 'test', workers=len(TASKS))
    tasks.register(setup, TASKS.SequencerTask1)

    # Start everything
    log.info('Starting sequencer.')
    tasks.run()
