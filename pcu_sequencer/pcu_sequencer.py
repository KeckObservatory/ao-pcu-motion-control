### pcu_sequencer.py : A document to contain the high-level sequencer code for all 5 named positions of the PCU
### Authors : Emily Ramey, Grace Jung
### Date : 11/22/21

from transitions import Machine, State

state_lookup = {
    'A': [0, 0, 0, 1],
    'B': [0, 0, 1, 0],
    'C': [0, 0, 1, 1],
    'D': [0, 1, 0, 0],
    'E': [0, 1, 0, 1],
}

state_names = ['A', 'B', 'C', 'D', 'E']

class RunPCU:
    
    states = []
    for name in state_names:
        states.append(State(name=name, on_enter='move_motor'))
    
    def __init__(self):
        
        self.machine = Machine(model = self, states = RunPCU.states,
                               before_state_change='home_Zstages',
                               initial = 'E')
    
    def home_Zstages(self):
        print("Homing Z stages")
    
    def move_motor(self):
        positions = state_lookup[self.state]
        print(f"Moving motor to {positions} in state {self.state}")