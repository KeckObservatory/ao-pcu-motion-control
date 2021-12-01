from epics import PV
from kPySequencer.Sequencer import Sequencer, PVDisconnectException, PVConnectException

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
        'enable': ':enable',
        'enableRb': ':enableRb',
#         'torque': ':enableTorque',
#         'torqueRb': 'enableTorqueRb',
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
                raise PVDisconnectException(f"Channel {pv.pvname} has disconnected.")
    
    def isEnabled(self):
        """ Checks whether the motor is enabled """
        # Software enable channel is backwards
        # Torque enable works fine
        return (not self.enableRb.get()) # and self.torqueRb.get()
    
    def enable(self):
        """ Enables the motor """
        self.check_connection()
        self.enable.set(0) # Enable software
        self.torque.set(1) # Enable torque
    
    def disable(self):
        """ Disables the motor """
        self.check_connection()
        self.torque.set(0) # Disable torque
        self.enable.set(1) # Disable software
    
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