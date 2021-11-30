import PCU_util as util

valid_motors = util.valid_motors

# Position class
class PCUPos():
    
    def __init__(self, pos_dict=None, **kwargs):
        """ 
        Initializes a position with a dictionary or with m# arguments.
        Valid motors that are not specified will be set to zero.
        """
        self.pos_dict = {}
        # Check inputs
        if pos_dict is None:
            pos_dict = kwargs
        
        # Load motor positions
        for m_name in valid_motors:
            # Set motor position, zero if not specified
            pos = pos_dict[m_name] if m_name in pos_dict else 0
            self.pos_dict[m_name] = pos
    
    def m_dict(self):
        """ Returns a dictionary of positions """
        return self.pos_dict
    
    def is_between(self, m_name, limits):
        """ Checks whether a motor position is within the limit array """
        if m_name not in valid_motors:
            raise ValueError(f"{m_name} is not a valid motor.")
        num = self.pos_dict[m_name]
        return num >= limits[0] and num <= limits[1]
    
    def in_limits(self, limits):
        """ Checks whether the position is within the given limits (dict) """
        # Add a check for extras in the limits dict
        for m_name in valid_motors:
            if m_name in limits and not is_between(m_name, limits[m_name]):
                    return False
        
        return True
    
    def in_hole(self, instrument):
        """ Determines whether a position is in the limits for the 'fiber' or 'mask' configurations """
        # Get the correct set of limits
        if instrument=='fiber':
            limits = fiber_limits
        elif instrument=='mask':
            limits = mask_limits
        else:
            print("Unknown limit type")
            return None

        return self.in_limits(limits)