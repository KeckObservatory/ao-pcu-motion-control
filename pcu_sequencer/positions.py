import PCU_util as util

# Position class
class PCUPos():
    
    valid_motors = ['m1', 'm2', 'm3', 'm4'] ### CHANGE THIS BEFORE USING
    fiber_limits = util.fiber_limits
    mask_limits = util.mask_limits
    
    def __init__(self, pos_dict=None, name=None, **kwargs):
        """ 
        Initializes a position with a dictionary or with m# arguments.
        Valid motors that are not specified will be set to zero.
        """
        self._mdict = {}
        # Check inputs
        if pos_dict is None:
            pos_dict = kwargs
        
        # Load motor positions
        for m_name in PCUPos.valid_motors:
            # Set motor position, zero if not specified
            pos = pos_dict[m_name] if m_name in pos_dict else 0
            self._mdict[m_name] = pos
            self.add_property(m_name)
        
        self.name = '(unnamed)' if name is None else name
    
    def add_property(self, m_name):
        """ Adds a property for individual motors """
        # Define setter function
        def setter(self, val):
            self._mdict[m_name] = val
        # Define getter function
        getter = lambda self: self._mdict[m_name]
        setattr(PCUPos, m_name, property(getter, setter))
    
    def __str__(self):
        string = f"Position {self.name}: ["
        string += ', '.join([f'{m}: {val}' for m,val in self._mdict.items()])
        string += ']'
        return string
    
    def __repr__(self):
        return str(self)
    
    def __add__(self, other):
        new_dict = {}
        for m_name, val in self.mdict.items():
            # Check for motor mismatch
            if m_name not in other.mdict: raise ValueError("Cannot add positions.")
            # Add to new dictionary
            new_dict[m_name] = val + other.mdict[m_name]
        return PCUPos(new_dict)
    
    @property
    def mdict(self):
        """ Returns a dictionary of positions """
        return self._mdict
    
    def is_between(self, m_name, limits):
        """ Checks whether a motor position is within the limit array """
        if m_name not in PCUPos.valid_motors:
            raise ValueError(f"{m_name} is not a valid motor.")
        num = self._mdict[m_name]
        return num >= limits[0] and num <= limits[1]
    
    def in_limits(self, limits):
        """ Checks whether the position is within the given limits (dict) """
        # Add a check for extras in the limits dict
        for m_name in PCUPos.valid_motors:
            if m_name in limits and not self.is_between(m_name, limits[m_name]):
                    return False
        
        return True
    
    def in_hole(self, instrument): # May need to move to sequencer.py
        """ Determines whether a position is in the limits for the 'fiber' or 'mask' configurations """
        # Get the correct set of limits
        if instrument=='fiber':
            limits = PCUPos.fiber_limits
        elif instrument=='mask':
            limits = PCUPos.mask_limits
        else:
            print("Unknown limit type")
            return None

        return self.in_limits(limits)
    
    def is_valid(self): # May need to move to sequencer.py
        """ Checks whether a position is valid or not """
        valid = True
        
        # Check for m3 collisions
        if 'm3' in PCUPos.valid_motors and self.m3 > 0:
            if not self.in_hole('mask'): valid = False
        # Check for m4 collisions
        if 'm4' in PCUPos.valid_motors and self.m4 > 0:
            if not self.in_hole('fiber'): valid = False
        
        ### TODO: Add a check for motor limits
        
        return valid
    
    def move_in_hole(self, other):
        """ Checks whether a move takes place in the hole of the k-rotator """
        # Check both are valid
        if not self.is_valid() or not other.is_valid():
            return False
        
        # Check for pinhole mask mini-moves
        if self.in_hole('mask') and other.in_hole('mask'):
            return True
        
        # Check for fiber mini-moves
        if self.in_hole('fiber') and other.in_hole('fiber'):
            return True
        
        return False