#!/kroot/rel/default/bin/kpython3
#
# kpython safely sets RELDIR, KROOT, LROOT, and PYTHONPATH before invoking
# the actual Python interpreter.

# Precision Calibration Unit IOC for the PI linear stages

import os
import coloredlogs, logging
import argparse
import collections
from enum import Enum, auto
import traceback
from concurrent import futures
from threading import Lock, Event

from softioc import softioc, builder

try:
    import epicscorelibs.path.pyepics
except ImportError:
    pass
import epics



class MotorEPICS_ACS:

    def __init__(self, prefix):
        self.enable    = epics.PV(prefix + ".CNEN")
        self.status    = epics.PV(prefix + ".STAT")
        self.spmg      = epics.PV(prefix + ".SPMG")
        self.pos       = epics.PV(prefix + ".VAL")
        self.posrb     = epics.PV(prefix + ".RBV")
        self.posval    = epics.PV(prefix + ".VAL")
        self.posvalrb  = epics.PV(prefix + ".RBV")
        self.encoder   = epics.PV(prefix + ".REP")
        self.jogf      = epics.PV(prefix + ".JOGF")
        self.jogr      = epics.PV(prefix + ".JOGR")
        self.vel       = epics.PV(prefix + ".VELO")
        self.accelrb   = epics.PV(prefix + ".ACCL")
        self.decelrb   = epics.PV(prefix + ".ACCL")
        self.limitmax  = epics.PV(prefix + ".HLM")
        self.limitmin  = epics.PV(prefix + ".LLM")
        self.home      = epics.PV(prefix + ".HOMR")
        self.motres    = epics.PV(prefix + ".MRES")
        self.encres    = epics.PV(prefix + ".ERES")
        self.mres       = self.motres.get()
        self.eres       = self.encres.get()


'''
From the ICD:    

Interface                EPICS channels      Keywords
mX device            	 :connected          NMCONNECT
mX status (r)        	 :status             NMSTATUS
mX error (r)         	 :error              NMERROR
mX reboot (w)        	 :reboot             NMREBOOT
Stage halt (w)           :halt               NMHALT
Stage Go (w)             :go                 NMGO
Stage named position (w) :pos                NMNPOS
Stage named position (r) :posrb              NMNPOS
Stage position value (w) :posval             NMPOS
Stage position value (r) :posvalrb           NMPOS
Stage jog (w)            :jog                NMJOG
Stage velocity (w)       :vel                NMVEL
Stage velocity (r)       :velrb              NMVEL
Stage acceleration (w)   :accel              NMACCEL
Stage acceleration (r)   :accelrb            NMACCEL
Stage deceleration (w)   :decel              NMDECEL
Stage deceleration (r)   :decelrb            NMDECEL
Stage limit, max (r)     :limitmax           NMLIMMAX
Stage limit, min (r)     :limitmin           NMLIMITMIN
Home		         :limitmin           NMHOME

where NM is motor name
'''

# Define the channels based on the ICD above.
class CHANNELS(Enum):
    """Assign enum values to the channels"""
    CONNECTED = auto()
    STATUS = auto()
    ERROR = auto()
    REBOOT = auto()
    HALT = auto()
    GO = auto()
    HOME = auto()
    POS = auto()
    POSRB = auto()
    POSVAL = auto()
    POSVALRB = auto()
    JOG = auto()
    VEL = auto()
    VELRB = auto()
    ACCEL = auto()
    ACCELRB = auto()
    DECEL = auto()
    DECELRB = auto()
    LIMITMIN = auto()
    LIMITMAX = auto()
    ENABLE = auto()
    ENABLERB = auto()

class ChannelCreateException(BaseException):
    """Exception to indicate a channel could not be created"""
    pass

class PCU_LINEAR:
    """
    Main class for an PCU IOC Linear stage
    """

    # These channels can be written by outside clients, and need a callback tied to them
    ioc_channels_callback = \
                   [('reb',       bool,  CHANNELS.REBOOT),  # Reboot the controller
                    ('halt',      bool,  CHANNELS.HALT),  # Halt the stage
                    ('go',        bool,  CHANNELS.GO),  # Restart the stage
                    ('pos',       str,   CHANNELS.POS),  # Set named position
                    ('posval',    float, CHANNELS.POSVAL),  # Set position, in mm
                    ('jog',       float, CHANNELS.JOG),  # Jog position, in mm (+/-)
                    ('vel',       float, CHANNELS.VEL),  # Set velocity, in mm/s
                    ('acc',       float, CHANNELS.ACCEL),  # Set acceleration, in mm/s^2
                    ('dec',       float, CHANNELS.DECEL),  # Set deceleration, in mm/s^2
                    ('home',      float, CHANNELS.HOME),  # Init stage
                    ('enable',    bool,  CHANNELS.ENABLE),  # Enable stage
                    ]

    # These channels are controlled strictly by the IOC and cannot be written by outside clients
    ioc_channels = [('connected', bool,  CHANNELS.CONNECTED), # Controller connected status
                    ('errRb',     str,   CHANNELS.ERROR), # Last stage error
                    ('posRb',     str,   CHANNELS.POSRB), # Position readback, as named position
                    ('posvalRb',  float, CHANNELS.POSVALRB), # Position readback, in mm
                    ('velRb',     float, CHANNELS.VELRB), # Velocity, in mm/s
                    ('accelRb',   float, CHANNELS.ACCELRB), # Acceleration, in mm/s^2
                    ('decelRb',   float, CHANNELS.DECELRB), # Deceleration, in mm/s^2
                    ('limmin',    float, CHANNELS.LIMITMIN), # Min limit, in mm
                    ('limmax',    float, CHANNELS.LIMITMAX), # Max limit, in mm
                    ('enableRb',  bool,  CHANNELS.ENABLERB),  # Enable stage status
                    ]

    # Stub out some named positions, their location (mm), and tolerance
    named_positions = {'HOME' : (0.0, 0.002),
                       '100': (100, 0.002),
                       }

    def __init__(self, prefix, motor, debug=False):
        self.debug = debug
        self.log = logging.getLogger(__name__)

        # Task properties
        self.tickrate = 0.1 # 0.01 # Run the loop at 100 Hz
        self.ready = Event()
        self.ready.clear()
        self._stop = Event()
        self._stop.clear()
        self.prefix = prefix
        self.motor = motor

        # PI device properties
        self.pidevice = None

        # EPICS interface properties
        self.channels = dict() # Uses the alias as the key
        self.channelAliases = dict() # Uses the channel name as the key to get the alias
        self.pending = collections.deque()

    def createChannel(self, name, recordtype, callbackmethod=None):
        """Create an EPICS channel database entry"""

        if callbackmethod is None:

            if recordtype is float:
                result = builder.aOut(name, initial_value=0.0)
            elif recordtype == int:
                result = builder.longOut(name, initial_value=0)
            elif recordtype == str:
                result = builder.stringOut(name, initial_value='')
            elif recordtype == bool:
                result = builder.boolOut(name, False, True, initial_value=False)
            elif type(recordtype) is type(Enum):
                fields = [x.name for x in recordtype]
                result = builder.mbbOut(name, *fields, initial_value=0)
            else:
                raise ChannelCreateException(f'Unknown channel type {recordtype} for {name}')

        else:

            if recordtype is float:
                result = builder.aOut(name, initial_value=0.0, validate=callbackmethod, always_update = True)
            elif recordtype == int:
                result = builder.longOut(name, initial_value=0, validate=callbackmethod, always_update = True)
            elif recordtype == str:
                result = builder.stringOut(name, initial_value='', validate=callbackmethod, always_update = True)
            elif recordtype == bool:
                result = builder.boolOut(name, False, True, initial_value=False, validate=callbackmethod, always_update = True)
            elif type(recordtype) is type(Enum):
                fields = [x.name for x in recordtype]
                result = builder.mbbOut(name, *fields, initial_value=0, validate=callbackmethod, always_update = True)
            else:
                raise ChannelCreateException(f'Unknown command channel type {recordtype} for {name}')

        return result

    def callback(self, pv, value):
        """Handle the callback from a channel changing"""
        self.log.debug(f'{pv.name} -> {value}')

        # Use the PV name to get the channel alias (which is an enum value)
        alias = self.channelAliases[pv.name]

        # Enqueue an I/O change event using the alias for the channel, making it easier to
        # match in the event processor.
        self.pending.append((alias, value))

        # Returning True here "approves" the value change to the channel
        return True

    def startIOC(self):
        """Create the channels with the supplied prefix"""

        # Make sure the IOC prefix ends with a colon
        if not self.prefix.endswith(':'):
            self.prefix = self.prefix + ':'

        try:
            # Build channels about the IOC itself
            for channel, type, alias in self.ioc_channels_callback:
                name = self.prefix + self.motor + ':' + channel
                self.log.info(f'Creating EPICS channel {name} with callback')
                chan = self.createChannel(name, type, self.callback)

                # Build the dictionaries of channels
                self.channelAliases[chan.name] = alias
                self.channels[alias] = chan

            # Build channels about the IOC itself
            for channel, type, alias in self.ioc_channels:
                name = self.prefix + self.motor + ':' + channel
                self.log.info(f'Creating EPICS channel {name}')
                chan = self.createChannel(name, type)

                # Build the dictionaries of channels
                self.channelAliases[chan.name] = alias
                self.channels[alias] = chan

            builder.LoadDatabase()

            # Load the IOC stats module
            #softioc.dbLoadDatabase('ioc.template',
            #                       '/usr/local/epics/default/modules/src/deviocstats/default/iocAdmin/Db',
            #                       f'IOCNAME={self.prefix}:stats,TODFORMAT=%m/%d/%Y %H:%M:%S')

            # Start the IOC running
            softioc.iocInit()

            # IOC is running, let the main thread run now
            self.log.debug('IOC thread started.')
            self.ready.set()

        except Exception as e:
            self.log.critical(f'Exception in IOC thread: {e}')
            return False

        # Finished
        return True

    def ioDriver(self):
        """Run the IO for the PI device."""

        # ---------------------------------------------------
        # Connect to the device, if needed
        if self.pidevice is None:
            self.log.info(f'connecting to %s'%(self.prefix + self.motor))
            self.pidevice = MotorEPICS_ACS(self.prefix + self.motor)

        # Set some channels that describe limits
        self.channels[CHANNELS.LIMITMIN].set(self.pidevice.limitmin.get())
        self.channels[CHANNELS.LIMITMAX].set(self.pidevice.limitmax.get())

        # ---------------------------------------------------
        # Process any changed channel values
        while len(self.pending):
            try:
                alias, value = self.pending.popleft()

                # Command the motion controller to reboot
                try:
                    # Home the device
                    if alias == CHANNELS.HOME:
                        self.pidevice.home.put(1)

                    # Halt the axis
                    elif alias == CHANNELS.HALT:
                        if value:
                            self.pidevice.spmg.put(0)
                            # Clear the channel on the way out
                            self.channels[CHANNELS.HALT].set(False)

                    # restart the axis
                    elif alias == CHANNELS.GO:
                        if value:
                            self.pidevice.spmg.put(3)
                            # Clear the channel on the way out
                            self.channels[CHANNELS.GO].set(False)

                    # Set position, named
                    elif alias == CHANNELS.ENABLE:
                        self.pidevice.enable.put(value)

                    # Set position, named
                    elif alias == CHANNELS.POS:
                        self.pidevice.posval.put(named_position[value]/(self.pidevice.mres)*(self.pidevice.eres*1e-3))

                    # Set position, in mm
                    elif alias == CHANNELS.POSVAL:
                        self.pidevice.posval.put(value/(self.pidevice.mres)*(self.pidevice.eres*1e-3))

                    # Jog position, in mm
                    elif alias == CHANNELS.JOG:
                        if value > 0:
                            self.pidevice.jogr(value/(self.pidevice.mres)*(self.pidevice.eres*1e-3))
                        elif value <0:
                            self.pidevice.jogf(value/(self.pidevice.mres)*(self.pidevice.eres*1e-3))

                        self.channels[CHANNELS.JOG].set(0)
                    else:
                        self.log.critical(f'No handler for event for channel {alias}')

                except BaseException as e:
                    self.log.critical(f'Exception processing command: {e}')
                    self.channels[CHANNELS.ERROR].set(e.args[0].encode('UTF-8'))

            except IndexError:
                break

        # ---------------------------------------------------
        # Read back the IO controller states
        try:
            #self.channels[CHANNELS.STATUS].set(self.pidevice.status.value)
            self.channels[CHANNELS.POSVALRB].set(self.pidevice.encoder.get()*self.pidevice.mres)
            self.channels[CHANNELS.VELRB].set(self.pidevice.vel.get())
            self.channels[CHANNELS.ACCELRB].set(self.pidevice.accelrb.get())
            self.channels[CHANNELS.DECELRB].set(self.pidevice.decelrb.get())
            self.channels[CHANNELS.ENABLERB].set(self.pidevice.enable.get())

        except BaseException as e:
            self.log.critical(f'Messaging to controller failed: {e}')

            self.pidevice = None

    @property
    def stopping(self):
        return self._stop.isSet()

    def stop(self):
        self._stop.set()

    def startDevice(self):
        """
        Main thread loop.
        """
        self.log.debug('Device thread waiting on IOC to start...')
        while not self.ready.wait(self.tickrate):
            pass

        self.log.debug('Device thread running.')
        while not self._stop.wait(self.tickrate):
            try:
                self.channels[CHANNELS.CONNECTED].set(True)

                # Run the driver IO once
                self.ioDriver()

            except Exception as e:
                self.log.critical(f'Exception during processing: {e}')
                self.log.critical(traceback.print_tb(e.__traceback__))

        self.channels[CHANNELS.CONNECTED].set(False)
        self.log.debug('Device thread stopped.')


if __name__ == "__main__":
    debug = False

    # -------------------------------------------------------------------------
    # Commandline arguments
    parser = argparse.ArgumentParser(description='PCU IOC: PI linear stage ACS motion controller')
    parser.add_argument('-d', '--debug', help='Enable debugging output', action='store_true')
    parser.add_argument('-ioc', '--ioc', help='IOC prefix (e.g. kN:ao:pcu:ln)', required=True, type=str, default='k1:ao:as:pcu:ln')
    parser.add_argument('-motor', '--motor', help='motor name (e.g. m1)', required=True, type=str, default='m1')
    parser.add_argument('-eport', '--eport', help='EPICS_CA_SERVER_PORT (e.g. 8601)', required=True, type=str, default='8601')
    args = parser.parse_args()

    # Get the debug argument first, as it drives our logging choices
    if args.debug:
        debug = True

    os.environ["EPICS_CA_SERVER_PORT"] = args.eport
    # -------------------------------------------------------------------------
    # Set up the base logger all threads will use, once we know the debug flag
    coloredlogs.DEFAULT_LOG_FORMAT = '%(asctime)s [%(levelname)s] %(message)s'
    coloredlogs.DEFAULT_DATE_FORMAT = '%Y-%m-%d %H:%M:%S.%f'
    if debug:
        coloredlogs.install(level='DEBUG')
    else:
        coloredlogs.install(level='INFO')
    log = logging.getLogger(__name__)

    if debug:
        log.info('Debug output enabled.')

    prefix = args.ioc
    log.info(f'Setting IOC channel name prefix to "{prefix}"')
    motor = args.motor
    log.info(f'Setting IOC channel name prefix to "{motor}"')

    # -------------------------------------------------------------------------
    # Instantiate the device, which will contain two threads: the main thread
    # for PI traffic, and the IOC thread
    device = PCU_LINEAR(prefix=prefix, motor=motor, debug=debug)

    log.info('Starting PCU Linear Stages IOC.')

    # Create a task pool to track our thread, set workers to match the number of threads
    pool = futures.ThreadPoolExecutor(thread_name_prefix=prefix, max_workers=2)
    try:
        # Start the thread
        iocthread = pool.submit(device.startIOC)
        devicethread = pool.submit(device.startDevice)

        # Wait for the thread to complete
        for r in futures.as_completed([iocthread, devicethread]):
            pool.shutdown(True)
            log.info('Shutdown: threads stopped normally.')

    except (SystemExit, KeyboardInterrupt):
        log.info('Exit/Interrupt: shutting down.')
        device.stop()
        pool.shutdown(False)

    log.info('Done.')
