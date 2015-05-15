# -*- coding: utf-8 -*-
"""

Filename: FuelOctobot.py
Author: Ryan L. Truby
Affiliation: Lewis Research Group, Harvard University
Data: 2015.01.19

Description:
    This code prints the downstream components of a "fancy" octobot. These 
    components include the actuators and all networks interconnecting them. 
    Additionally, the fuel reservoir is printed upstream of a logic module.
    All interconnects to the logic module are also printed. This code is an
    adaptation based off Daniel Fitzgerald's script "FuelSpider.py". 
    
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
#import FancyOctobot
import FancyOctobot2
import LogicModule
import OctobotLogicModule

# Define the export file that will hold the generated G code
import os
cur_filepath = os.path.dirname(os.path.realpath(__file__))
splitted = cur_filepath.split('/')
n = len(splitted)
cur_dir = ''.join((dir+'/') for dir in splitted[:n-1])
exportFileDir = cur_dir + "soft_robotics_octobot/FuelOctobot.pgm"
print "Exporting to file " + str(exportFileDir)

e3DPGlobals.init_G(exportFileDir)

'''
    Note from RTruby:
        On Jan 19, 2015, I went through all of Dan Fitzgerald's code, and made
        sure I understood anything up until this comment.
        
        GO BACK AND REVIEW THE CODE BELOW AND WHAT IT'S DOING.
'''

def print_fuel_spider(flow_connector_height_abs, centerline_x, flow_connectors_centerline_offset, flow_connectors_back_y):    
    ##plenum_bottom_height = -3 #FancyOctobot2.mold_top - 7.6; -3 for newer octobot molds
    plenum_bottom_height = -6 # for deepened fuel reservoir on octobot mold
    plenum_top_height = FancyOctobot2.mold_top - 5
    ##plenum_meander_print_height = (plenum_top_height + plenum_bottom_height)/2.0
    ##plenum_meander_print_height = -6 # added 2015.02.03, changed from 6.9 to 6 on 2015.02.04
    ##plenum_meander_print_height = -5.5 #added 2015.02.06
    plenum_meander_print_height = -5.5
    #line below commented out on 2015.02.03
    #module_top_print_height = -1 # max(LogicModule.module_hole_top+0.2,flow_connector_height_abs) # TODO: Compensate for this if it ends up being below the flow_connector_height_abs
    ##module_top_print_height = -2.3 
    module_top_print_height = -0.45 # added 2015.02.20 for new octobot mold   
    ##module_print_height_back = -4.3 # 2015.02.03 was -1
    ##module_print_height_back = -3.5 # changed 2015.02.06
    module_print_height_back = -0.45 # added 2015.02.20
    e3DPGlobals.g.absolute()
    
    LogicModule.init(centerline = centerline_x, offset = flow_connectors_centerline_offset, print_height = module_top_print_height)
    OctobotLogicModule.hook_module_into_fancy_spider()
        
       
    def print_plenum_meander(Left):
        x_offset_mult = (-1 if Left else 1)
        
        height_offset = 1.5
        
        # Print the fuel reservoirs, starting at the check valves. This code was changed for D-90.
        LogicModule.interface_with_hole(Left = Left, Front = False, to_cliff = False, Valve = False)
        e3DMatrixPrinting.travel_mode()
        LogicModule.interface_with_hole(Left = Left, Front = False, to_cliff = False, Valve = True)
        e3DMatrixPrinting.move_z_abs(module_print_height_back - height_offset, vertical_travel_speed = e3DMatrixPrinting.default_z_drag_speed)
        e3DMatrixPrinting.travel_mode()
        
        meander_front_y = -41 # 2015.02.03 - switched from -45, -40 was too close
        meander_front_y = -44.5 # for new Octobot molds used in D-90, 2015.04.21
        meander_back_y = -45 # 2015.02.03 - switched from -50
        meander_back_y = -47.5 # for new Octobot molds used in D-90, 2015.04.21
        ##meander_inner_x_offset = 3 # was originally 0.75 in Dan's code
        meander_inner_x_offset = 1.5 #added for D-80
        connection_overlap = 0.0   # what is this? it was originally in Dan's code as 0.5 originally
        meander_print_speed = 0.5 # switched from 0.6 on 2014.02.06
        meander_connection_dwell_time = 1
        
        e3DPGlobals.g.abs_move(x=centerline_x+x_offset_mult*meander_inner_x_offset, y=meander_back_y-connection_overlap)
        e3DMatrixPrinting.print_mode(print_height_abs = plenum_meander_print_height, print_speed = meander_print_speed)
        safe_hole_connection_x_offset = meander_inner_x_offset
        ##e3DMatrixPrinting.move_z_abs(height = plenum_meander_print_height) # redundant?
        e3DPGlobals.g.feed(e3DMatrixPrinting.default_print_speed)
        e3DPGlobals.g.abs_move(y = meander_front_y)
        e3DPGlobals.g.abs_move(x = centerline_x+x_offset_mult*meander_inner_x_offset)
        e3DMatrixPrinting.move_z_abs(height = module_print_height_back - height_offset)
        e3DPGlobals.g.feed(e3DMatrixPrinting.default_print_speed)
        
        hole_pos = (LogicModule.check_valve_left_hole_pos if Left else LogicModule.check_valve_right_hole_pos)
        e3DPGlobals.g.abs_move(x=hole_pos[0], y=hole_pos[1]) # move above the hole
        e3DPGlobals.g.dwell(meander_connection_dwell_time)
        e3DMatrixPrinting.travel_mode() 
          
        # Now, connect the fule line to the inlets to the oscillator:
        e3DPGlobals.g.abs_move(x=centerline_x+x_offset_mult*meander_inner_x_offset, y=meander_back_y-connection_overlap)
        e3DMatrixPrinting.print_mode(print_height_abs = plenum_meander_print_height, print_speed = meander_print_speed)
        e3DPGlobals.g.feed(e3DMatrixPrinting.default_print_speed)
        e3DPGlobals.g.abs_move(x = centerline_x + x_offset_mult * meander_inner_x_offset * 2)
        e3DPGlobals.g.abs_move(y = meander_front_y)
        e3DMatrixPrinting.move_z_abs(height = module_print_height_back)
        e3DPGlobals.g.feed(e3DMatrixPrinting.default_print_speed)
        ##e3DPGlobals.g.abs_move(y = LogicModule.module_back_edge_y)
        
        hole_pos = (LogicModule.back_left_hole_pos if Left else LogicModule.back_right_hole_pos)
        e3DPGlobals.g.abs_move(x = hole_pos[0], y = hole_pos[1])
        e3DPGlobals.g.dwell(meander_connection_dwell_time)
        e3DMatrixPrinting.travel_mode()
        
                        
    print_plenum_meander(Left=True)
    print_plenum_meander(Left=False)

# SET THESE: LEG ACTUATOR ECOFLEX ZEROS
left_zero = -58.725
right_zero = -58.8127
# left_zeros and right_zeros are needed for FancyOctobot2
left_zeros = [-57.2102, -57.1204, -57.1126, -57.1] # L1, L2, L3, L4, added 20150409
right_zeros = [-57.301, -57.1908, -57.1744, -57.1608] # R1, R2, R3, R4, added 20150409

#FancyOctobot was used up until Experiment D-90 (2015.04.21)
#FancyOctobot2.print_robot(ecoflex_zero_left = left_zero, ecoflex_zero_right = right_zero, func_print_internal_soft_logic=print_fuel_spider)     
FancyOctobot2.print_robot(ecoflex_zero_left = left_zeros, ecoflex_zero_right = right_zeros, func_print_internal_soft_logic=print_fuel_spider)
#The line below commented out on 2014.09.10 by RTruby, for Experiment C-95
e3DPGlobals.g.view('matplotlib')
#e3DPGlobals.g.view()
e3DPGlobals.g.teardown()

