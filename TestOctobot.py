# -*- coding: utf-8 -*-
"""

Filename: TestOctobot.py
Author: Ryan L. Truby
Affiliation: Lewis Research Group, Harvard University
Data: 2015.03.20

Description:
    This code prints only the downstream components of a "fancy" octobot. These 
    components include the actuators and all networks interconnecting them. This 
    code is based off FuelOctobot.py. "TestOctobots" will be used to test 
    Octobot performance sans logic modules.  
    
"""

from mecode import G
from math import sqrt

import numpy as np
import matplotlib.pyplot as plt

# Soft robot printing libraries
import e3DMatrixPrinting
import MultiMaterial

# Soft robot features
import Actuators
import e3DPGlobals
import FancyOctobot
import FancyOctobot2
import LogicModule
import OctobotLogicModule

# Define the export file that will hold the generated G code
import os
cur_filepath = os.path.dirname(os.path.realpath(__file__))
splitted = cur_filepath.split('/')
n = len(splitted)
cur_dir = ''.join((dir+'/') for dir in splitted[:n-1])
exportFileDir = cur_dir + "soft_robotics_octobot/TestOctobot.pgm"
print "Exporting to file " + str(exportFileDir)

e3DPGlobals.init_G(exportFileDir)

'''
    Note from RTruby:
        On Jan 19, 2015, I went through all of Dan Fitzgerald's code, and made
        sure I understood anything up until this comment.
        
        GO BACK AND REVIEW THE CODE BELOW AND WHAT IT'S DOING.
'''

def print_test_spider(flow_connector_height_abs, centerline_x, flow_connectors_centerline_offset, flow_connectors_back_y):    
    '''
    plenum_bottom_height = -3 #FancyOctobot.mold_top - 7.6; -3 for newer octobot molds
    plenum_bottom_height = -6 # for deepened fuel reservoir on octobot mold
    
    plenum_top_height = FancyOctobot.mold_top - 5
    plenum_meander_print_height = (plenum_top_height + plenum_bottom_height)/2.0
    plenum_meander_print_height = -6 # added 2015.02.03, changed from 6.9 to 6 on 2015.02.04
    plenum_meander_print_height = -5.5 #added 2015.02.06
    plenum_meander_print_height = -5.5
    #line below commented out on 2015.02.03
    #module_top_print_height = -1 # max(LogicModule.module_hole_top+0.2,flow_connector_height_abs) # TODO: Compensate for this if it ends up being below the flow_connector_height_abs
    module_top_print_height = -2.3 
    module_top_print_height = -0.45 # added 2015.02.20 for new octobot mold   
    module_print_height_back = -4.3 # 2015.02.03 was -1
    module_print_height_back = -3.5 # changed 2015.02.06
    module_print_height_back = -0.45 # added 2015.02.20
    '''
    module_top_print_height = -0.45 # added 2015.02.20 for new octobot mold
    module_print_height_back = -0.45 # added 2015.02.20
    
    e3DPGlobals.g.absolute()
    
    fuel_line_length = 1
    needle_insertion_length = 3
    needle_insertion_separation = 0
    
    LogicModule.init(centerline = centerline_x, offset = flow_connectors_centerline_offset, print_height = module_top_print_height)
    OctobotLogicModule.print_pressureChamber_for_test_octobot(Left=True, fuel_line = fuel_line_length, needle_insertion = needle_insertion_length, spacer = needle_insertion_separation/2)
    OctobotLogicModule.print_pressureChamber_for_test_octobot(Left=False, fuel_line = fuel_line_length, needle_insertion = needle_insertion_length, spacer = needle_insertion_separation/2)
    
# SET THESE: LEG ACTUATOR ECOFLEX ZEROS
#left_zero = -58.781
#right_zero = -58.7612


left_zeros = [-57.3652, -57.2957, -57.3389, -57.3503] # L1, L2, L3, L4, added 20150409
right_zeros = [-57.503, -57.4661, -57.4969, -57.5292] # R1, R2, R3, R4, added 20150409

#FancyOctobot.print_robot(ecoflex_zero_left = left_zero, ecoflex_zero_right = right_zero, func_print_internal_soft_logic=print_test_spider)     
# added 20150409
FancyOctobot2.print_robot(ecoflex_zero_left = left_zeros, ecoflex_zero_right = right_zeros, func_print_internal_soft_logic=print_test_spider)

#The line below commented out on 2014.09.10 by RTruby, for Experiment C-95
e3DPGlobals.g.view('mayavi')
#e3DPGlobals.g.view()
e3DPGlobals.g.teardown()

