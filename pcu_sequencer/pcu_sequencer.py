### pcu_sequencer.py : A document to contain the high-level sequencer code for all 5 named positions of the PCU
### Authors : Emily Ramey, Grace Jung
### Date : 11/22/21

from transitions import Machine, State
import yaml

yaml_file = "PCU_configurations.yaml"

with open(yaml_file) as file:
    state_lookup = yaml.load(file, Loader=yaml.FullLoader)

class RunPCU:
    
    states = []
    for name in state_lookup:
        states.append(State(name=name, on_enter='move_motors'))
    
    def __init__(self):
        
        self.machine = Machine(model = self, states = RunPCU.states,
                               before_state_change='home_Zstages',
                               initial = 'telescope')
    
    def home_Zstages(self):
        print("Homing Z stages")
    
    def move_motors(self):
        positions = state_lookup[self.state]
        print(f"Moving motor to {positions} in state {self.state}")