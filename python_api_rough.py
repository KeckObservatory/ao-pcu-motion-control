import epics
# from epics import PV,
    # importing PV to create PV "objects" for future, continued reference

# Default running of PCU components (should be as simple as possible, getting it to run) (Abstract class?)
class PCU:
    """
    For API, should users have ability to manually add new stages? (probably? but ignore for now)
        - Or should this code have the stage information stored by default? (since current hardware is known)
        - TODO: get the "IOC Database" - this contains records that host the PVs
        - TODO: look into difference (implementation, features, our own hardware, etc.) difference between PV vs Device for code
        - TODO: where to contain the collision detection? within the stage class or PCU class? (leaning towards stage)
        - TODO: split up code into different files to "contain" different parts of running code/calling it

    Idea:
    """
    self.stages = []
    self.rotators = []

    def __init__(self)
        stages = [] # TODO: Store the different State objects (how implement? Hardcode?
        rotators = []
        # [stage_x, stage_y, stage_z1, stage_z2]
        # [rotator]
        #for i in range(4):
        #    stages.append(Stage)

    # Methods to return information about PCU
    def get_PCU_components(self): # Return the PCU stages (what information do I want returned?)
        #for i in self.stages:
        # [] store and return information about pcu components?
        return self.stages + self.rotators

    def print_full_info(self): # Returns full (current) status information of all stages in PCU
        for stage in stages:
            print(stage.status_info()) # Prints info
            # <outfile> add stage.status_info <- Should we PORT/SAVE the information to a different location for testing, record keeping, etc?
        for rotator in rotators:

    # Control the stages and proceed in continuous process (default run of the PCU)
    def run_default(self):


"""
General thoughts/characteristics to look out for, for each PCU part:

stage: "X-Stage"
    - Component Name: PI L-412
stage: "Y-Stage"
    - Component Name: PI L-412
stage: "Z-Stage" (2 of them) - So will have to distinguish between two, since different movements
    - Component Name: PI L-406
stage: "Z-Stage"
    - Component Name: PI L-406
rotator: "PCU Rotator"
    - Component Name: PI U-651
"""

# The different stages/pieces to be moved
"""
TODO: Make stage into an abstract class (?), then the different stages of the PCU into versions of the class...
    or create one stage class with diferent instances/objects of the class?

    Use Device module at least. Or Motor Module (Epics Motor Device). Either way, extend the classes.
    ^ Actually, still unsure of basic running of PVs versus using Devices (due to nature of EPICS and its whole PV vs object-oriented programming)


"""
class Stage:
    # The stage object it references
    def __init__(self, title, motor_value) #, d):
        stage_motor = epics.Motor("motor_value") # TODO: Fill in the quotation with the respective 'XXX:m1.VAL' (How is this attained? When hardware is physically connected to the local subnet?)
        print("Stage " + title + " is initialized.") # When an instance of the stage is created (conceptually within the PCU, then should print this confirmation)
        #direction = d # <- Better way to organize x, y, z stage?

    # Methods regarding status the PCU stage
    def get_location(self):
        return stage_motor.get()

    def status_info(self):
        return stage_motor.info # Find correct syntax

    # Methods regarding controlling the PCU stage
    def move_to(self, location) #(self, x, y, z): # I think it's best to split this into sub-methods for abstraction?
        # Call collision detection here?
        # Move x, check for collisions/errors

        # Move y, check for collisions/errors

        # Move z, check for collisions/errors


class StageX(Stage):
    """
    Implements Stage class for x direction stage. Use PyEpics Epics Motor module.
    """
    def move_to(self, location):
        motor.move(location, wait=True)
class StageY(Stage):
    """
    Implements Stage class for y direction stage. Use PyEpics Epics Motor module.
    """
    def move_to(self, location):
        motor.move(location, wait=True)
class StageZ(Stage):
    """
    Implements Stage class for z direction stage. Use PyEpics Epics Motor module.
    """
    def move_to(self, location):
        motor.move(location, wait=True)


# Similar to Stage implemenation, but for rotation (think about what could be different?)
class Rotator:
    # The PV object it references
    def __init__(self, title, pv_value):
        rotator_motor = ("pv_value")
        print("Rotator " + title + " is initialized.")

    # Methods regarding status the PCU rotator
    def get_location(self):
        return rotator_motor.get()

    def status_info(self):
        return rotator_motor.info # Find correct syntax

    def rotate(self, r): # TODO: Find the correct terminology for this

# Controllers status checking -> Collision detection, etc.
    # This can be handled by PyEpics and Epics itself, but HOW do we define what a collision is from the hardware/computer point of view (where to set the hardware boundaries/limitations?)
        # - Would this be hardcoded since know dimensions of PCU hardware?
        # - Would require in-lab testing and seeing if collisions do happen

# Talk to the physical controllers and move them around (manually, by inputting coordinates?)
    # Two ways I'm considering: default, where all the values are known and pre-set VS wanting to specialize the PCU movement for any reason (future testing, new algorithms, etc.)
    # 1) So, this will need some abstract class or "default outline" that different PCU "usages" can base their program design off of

    # 2) Create own version of where to move equipment (personalized)




# Testing purposes (TODO)
