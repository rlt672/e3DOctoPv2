from aerotech_automator import AerotechAutomator
from mecode import G

g = G()

# This dictionary contains all the information about multiple substrates
#substrates = {
#    'slide1': { # We need to name substrates so we can refer to them later
#        'origin': (176, 112), # Lower left of substrate (or point anywhere on substrate if size='auto')
#        'size': 'auto', # Size of substrate. If set to 'auto', the size is detected automatically
#        'profile': True, # Whether or not to profile this substrate
#        'profile-spacing': (25, 25),# Spacing between profile points
#        },
#}
axes = ['A', 'B'] #The axes that will be used in this experiment

# Create the AerotechAutomator instance with axes and substrate info
automator = AerotechAutomator(
    axes=axes,
    #substrates=substrates,
)

automator.setup() # setup() method must be called to connect to the printer and sensors
#automator.automate() # this call performs all the steps for a full automation

automator.zero_all_nozzles()
automator.find_alignment_grooves()
automator.find_z_ref()
automator.go_to_heaven()

#------------------------------------------------------------------------------
# RTruby, 2014.09.10 - Code below commented out during Experiment C-95:
#
#automator.save_state('automation_values.json') # save all the collected data to a file
#
# Code below added when previous code was commented out:
automator.save_state(r"C:\Users\Lewis Group\Documents\GitHub\aerotech_automation\automation_values.json")
#-----------------------------------------------------------------------------
automator.teardown() # disconnect from the printer and sensors