### temp_ioc.py : A temporary document for writing the pcu ioc code that
### subclasses the IOC class in the Keck Sequencer.py.
### Code from this file will be moved to and integrated with the pcu_sequencer.py

class PCUIOC(IOC):
    """
    IOC subclass that manages the IOC for the PCU.
    """
    def __init__(self):
        super().__init__()

    def registerFloat(self, name, initial_value=0.0):
        """
        Register a float channel with this IOC for use.

        :param name: Name of the channel.
        :param initial_value: Initial value to set the channel.
        :return: Reference to the channel.
        """
        if self.running:
            raise self.IOCStartException(F'Cannot create channel {name} after IOC has started!')

        self._log.debug(f'[IOC] Creating sequencer float channel {name}')
        channel = builder.builder.aOut(name, initial_value=initial_value)
        self.channels[name] = channel
        return channel

    def registerPCUChannel(self, name, initial_value=False):
        super().registerBool(name, initial_value)

    def registerMotorChannel(self, name, initial_value=0):
        super().registerLong(name, initial_value)
        # TODO: Or use the following?
        super().registerFloat(name, initial_value)

# Sequencer.py for pcu

# TODO: Which IOC channels to keep?
# ioc_channels = [('connected', bool,  CHANNELS.CONNECTED), # Controller connected status
#                 ('errRb',     str,   CHANNELS.ERROR), # Last stage error
#                 ('posRb',     str,   CHANNELS.POSRB), # Position readback, as named position
#                 ('velRb',     float, CHANNELS.VELRB), # Velocity, in mm/s
#                 ('accelRb',   float, CHANNELS.ACCELRB), # Acceleration, in mm/s^2
#                 ('decelRb',   float, CHANNELS.DECELRB), # Deceleration, in mm/s^2
#                 ('enableRb',  bool,  CHANNELS.ENABLERB),  # Enable stage status
#                 ]

def __init__(self, prefix, tickrate=0.5):
    # TODO: Check integration with IOC
    # TODO: Rename 'get_pattern' and 'set_pattern' to 'get_prefix' and 'set_prefix' ?
    pcu_ioc = PCUIOC()

    # self.prepare(PCUStates) handles creating the channels for the transition state machine
    # ---- Creates the channels for the PCU positions. Motor channels already exist.

    super().__init__(prefix, pcu_ioc, tickrate)



    self.prepare(PCUStates)
    class IOC():
    """
    Container class for an EPICS IOC that is shared amongst all the sequencer instances in this process.
    This class is a singleton.
    """
    _log = logging.getLogger('')

    class IOCStartException(BaseException):
        """Exception to indicate a channel was created after the IOC was started"""
        pass

    # Singleton instantiation
    _instance = None
    def __new__(cls, *args, **kwargs):
        if not cls._instance:
            cls._instance = super(IOC, cls).__new__(cls, *args, **kwargs)
        return cls._instance

    def __init__(self):

        # Keep a catalog of the channels this IOC is maintaining.
        self.channels = dict()
        self.running = False

    def registerBool(self, name, initial_value=False):
        """
        Register a boolean channel with this IOC for use.

        :param name: Name of the channel.
        :param initial_value: Initial value to set the channel.
        :return: Reference to the channel.
        """
        if self.running:
            raise self.IOCStartException(F'Cannot create channel {name} after IOC has started!')

        self._log.debug(f'[IOC] Creating sequencer boolean channel {name}')
        channel = builder.boolOut(name, False, True, initial_value=initial_value)
        self.channels[name] = channel
        return channel

    def registerLong(self, name, initial_value=0):
        """
        Register a long integer channel with this IOC for use.

        :param name: Name of the channel.
        :param initial_value: Initial value to set the channel.
        :return: Reference to the channel.
        """
        if self.running:
            raise self.IOCStartException(F'Cannot create channel {name} after IOC has started!')

        self._log.debug(f'[IOC] Creating sequencer long channel {name}')
        channel = builder.longOut(name, initial_value=initial_value)
        self.channels[name] = channel
        return channel

    def registerString(self, name, initial_value=''):
        """
        Register a string channel with this IOC for use.

        :param name: Name of the channel.
        :param initial_value: Initial value to set the channel.
        :return: Reference to the channel.
        """
        if self.running:
            raise self.IOCStartException(F'Cannot create channel {name} after IOC has started!')

        self._log.debug(f'[IOC] Creating sequencer string channel {name}')
        channel = builder.stringOut(name, initial_value=initial_value)
        self.channels[name] = channel
        return channel

    def registerEnum(self, name, fields, initial_value=0):
        """
        Register an enum channel with this IOC for use.  This enum is limited to 16 states, due to how mbbout works
        in EPICS.

        :param name: Name of the channel.
        :param fields: List of tuples such as:
            ('OK', 0),
            ('FAILING', 1),
            ('FAILED', 2)

        :param initial_value: Initial value to set the channel.
        :return: Reference to the channel.
        """
        if self.running:
            raise self.IOCStartException(F'Cannot create channel {name} after IOC has started!')

        self._log.debug(f'[IOC] Creating sequencer enum channel {name}')
        channel = builder.mbbOut(name, *fields, initial_value=initial_value)
        self.channels[name] = channel
        return channel

    def registerWaveform(self, name, initial_value=None, length=0):
        """
        Register a waveform channel with this IOC for use.

        :param name: Name of the channel.
        :param initial_value: Initial value to set the channel.
        :param length: Max length of the array, else we will use the initial value to set the length.
        :return: Reference to the channel.
        """

        if self.running:
            raise self.IOCStartException(F'Cannot create channel {name} after IOC has started!')

        self._log.debug(f'[IOC] Creating sequencer string channel {name}')
        if initial_value is None:
            if length > 0:
                channel = builder.WaveformOut(name, length=length)
            else:
                # Defaults to 40, the standard EPICS max string length.
                channel = builder.WaveformOut(name, length=40)
        else:
            channel = builder.WaveformOut(name, initial_value=initial_value)

        self.channels[name] = channel
        return channel

    def start(self):

        # Only start the IOC once!  Multiple sequencers will be calling this, only allow one to actually start it.
        if self.running:
            return

        self.running = True
        self._log.debug(f'[IOC] Sequencer IOC starting.')

        # Load the DB with all our channels
        builder.LoadDatabase()

        # Load the IOC stats module
        # softioc.dbLoadDatabase('ioc.template','/usr/local/epics/default/modules/src/deviocstats/default/iocAdmin/Db','IOCNAME=k0:asim:stats,TODFORMAT=%m/%d/%Y %H:%M:%S')

        # Start the IOC itself, this will return when the IOC is terminated
        softioc.iocInit()
