import PCU_util as util

# Position class
class PCUPos():
    
    @staticmethod
    def from_dict(pos_dict):
        """ Returns a dictionary of PCUPos objects """
        new_dict = {}
        for name, pos in pos_dict.items():
            new_dict[name] = PCUPos(pos, name=name)
        return new_dict
    
    @staticmethod
    def load_motors():
        """ Loads motor info into static class variables """
        motor_info = util.load_motors()
        for key, val in motor_info.items():
            setattr(PCUPos, key, val)
    
    def __init__(self, pos_dict=None, name=None, **kwargs):
        """ 
        Initializes a position with a dictionary or with m# arguments.
        Valid motors that are not specified will be set to zero.
        """
        
        # Make a dictionary for motor positions
        self._mdict = {}
        # Check inputs
        if pos_dict is None:
            pos_dict = kwargs
        
        # Load motor positions
        for m_name in self.valid_motors:
            # Set motor position - will be None if not specified
            pos = pos_dict[m_name] if m_name in pos_dict else None
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
        string = f"PCUPos {self.name}: ["
        string += ', '.join([f'{m}: {val}' for m,val in self._mdict.items()])
        string += ']'
        return string
    
    def __repr__(self):
        return str(self)
    
    def __add__(self, other): # TODO: change this function to only accept PCUMoves
        """ Add PCUPos to PCUMove """
        if not type(other)==PCUMove:
            raise ValueError("Can only add PCUMove to PCUPos")
        
        new_dict = self.mdict.copy()
        for m_name, val in other.mdict.items():
            if val is not None:
                if other.type == 'relative': new_dict[m_name] += val
                elif other.type == 'absolute': new_dict[m_name] = val
        
        return PCUPos(new_dict)
    
    def __neg__(self):
        if type(self)==PCUMove and not self.type=='relative':
            raise ValueError("Can't negate absolute move")
        move = PCUMove()
        for m_name, val in self.mdict.items():
            if val is not None:
                move[m_name] = -val
        return move
    
    def __sub__(self, other):
        if type(other)==PCUMove:
            raise ValueError("Can only subtract PCUPos, not PCUMove.")
        return self + (-other)
    
    def __eq__(self, other):
        return self.mdict == other.mdict
    
    def __getitem__(self, key):
        if key not in self.valid_motors:
            raise ValueError(f"{key} not in valid motors")
        return self.mdict[key]
    
    def __setitem__(self, key, val):
        if key not in self.valid_motors:
            raise ValueError(f"{key} not in valid motors")
        self.mdict[key] = val
    
    def __contains__(self, key):
        return key in self.mdict
    
    def keys(self):
        return self.mdict.keys()
    
    def items(self):
        return self.mdict.items()
    
    def values(self):
        return self.mdict.values()
    
    def __iter__(self):
        return iter(self.mdict)
    
    @property
    def mdict(self):
        """ Returns a dictionary of positions """
        return self._mdict
    
    def copy(self):
        return PCUPos(self.mdict)
    
    def is_between(self, m_name, limits):
        """ Checks whether a motor position is within the limit array """
        if m_name not in PCUPos.valid_motors:
            raise ValueError(f"{m_name} is not a valid motor.")
        num = self[m_name]
        return num >= limits[0] and num <= limits[1]
    
    def in_limits(self, limits):
        """ Checks whether the position is within the given limits (dict) """
        # Add a check for extras in the limits dict
        for m_name in self.valid_motors:
            if m_name in limits and not self.is_between(m_name, limits[m_name]):
                    return False
        
        return True
    
    def in_circle(self, center, radius):
        """ Checks whether the position is in the circle described by limits """
        if not ('m1' in self.mdict and 'm2' in self.mdict):
            return False
        elif self.m1 is None or self.m2 is None:
            return False
        else: return (self.m1-center['m1'])**2 + (self.m2-center['m2'])**2 < radius**2
    
    def in_hole(self, instrument, check_rad=None): # May need to move to sequencer.py
        """ Determines whether a position is in the limits for the 'fiber' or 'mask' configurations """
        # Get the correct set of limits
        if instrument=='fiber':
            center = self.fiber_center
            radius = self.safe_radius['fiber']
        elif instrument=='mask':
            center = self.mask_center
            radius = self.safe_radius['mask']
        else:
            print("Unknown limit type")
            return None
        
        if check_rad is not None:
            radius = check_rad

        return self.in_circle(center, radius)
    
    def fiber_extended(self):
        if not 'm4' in self.mdict: return False
        else: return self.m4 > 0
    
    def mask_extended(self):
        if not 'm3' in self.mdict: return False
        else: return self.m3 > 0
    
    def is_valid(self): # May need to move to sequencer.py
        """ Checks whether a position is valid or not """
        for m_name, val in self.mdict.items():
            if val is None: return False
        
        # Check for m3 collisions
        if 'm3' in PCUPos.valid_motors and self.m3 > 0:
            if not self.in_hole('mask'): return False
        # Check for m4 collisions
        if 'm4' in PCUPos.valid_motors and self.m4 > 0:
            if not self.in_hole('fiber'): return False
        
        ### Check for motor limits
        if not self.in_limits(self.motor_limits):
            return False
        
        return True
    
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
    
    def motor_in_position(self, m_name, pos):
        """ Checks whether a motor (m_name) is in position (m_dest) """
        # Get PV getter for motor
        motor = self.motors[m_name]
        
        # Compare to destination within tolerance, return False if not reached
        t = self.tolerance[m_name]
        # Lower and upper limits
        in_pos = self.is_between(m_name, pos-t, pos+t)
        # Return whether the given motor is in position
        return in_pos

class PCUMove(PCUPos):
    """ Like a position, but with more support for None values """
    
    def __init__(self, move_type='relative', pos_dict=None, **kwargs):
        """ Initializes a move with 'absolute' or 'relative' type """
        super().__init__(pos_dict, name='PCUMove', **kwargs)
        if move_type not in ['absolute', 'relative']:
            raise ValueError("Move type must be 'absolute' or 'relative'")
        self.type = move_type
    
    def __bool__(self):
        """ True if dict contains values """
        for m_name in self.mdict:
            if m_name != None: return True
        return False
    
    def __add__(self, other): # TODO: change this function to only accept PCUMoves
        """ Add PCUPos to PCUMove """
        if not type(other)==PCUPos:
            raise ValueError("Can only add PCUMove to PCUPos")
        
        return other + self
    
    def is_valid(self, start_pos):
        """ Moves are not valid on their own """
        return super().is_valid(start_pos+self)
    
    def __str__(self):
        add_ons = {m:("+" if self.type=='relative' and val>0 else "") 
                   for m,val in self._mdict.items() if val is not None}
        string = f"{self.type.capitalize()} {self.name}: ["
        string += ', '.join([f'{m}->{add_ons[m]}{val}' 
                             for m,val in self._mdict.items() if val is not None])
        string += ']'
        return string
    
    def __repr__(self):
        return str(self)
    
    def items(self):
        real_dict = {key:val for key, val in self.items() if val is not None}
        return real_dict.items()
    
    def keys(self):
        real_dict = {key:val for key, val in self.items() if val is not None}
        return real_dict.keys()
    
    def values(self):
        real_dict = {key:val for key, val in self.items() if val is not None}
        return real_dict.values()
    
    def __iter__(self):
        real_dict = {key:val for key, val in self.items() if val is not None}
        return iter(real_dict)
    
    def __contains__(self, key):
        return key in self.mdict and self.mdict[key] is not None
    
    @property
    def xy(self):
        """ Returns a move with only the XY-components """
        return PCUMove(move_type = self.type, 
                       pos_dict = {key:val for key, val in self.mdict.items()
                        if key in ['m1', 'm2']})
    
    @property
    def z(self):
        """ Returns a move with only the Z-components """
        return PCUMove(move_type = self.type, 
                       pos_dict = {key:val for key, val in self.mdict.items() 
                        if key in ['m3', 'm4']})

# Load motor info from config file
PCUPos.load_motors()