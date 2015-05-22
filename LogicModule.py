# -*- coding: utf-8 -*-
"""

Filename: LogicModule.py
Author: Ryan L. Truby
Affiliation: Lewis Research Group, Harvard University
Date: 2015.01.19

Description:
    This library contains mecode functions for printing into logic modules. 
    This code is an adaptation of Dan Fitzgerald's script "LogicModule.py". 
       
2015.05.15:
    This file copied and pasted to "e3DOctoPv2" to code up a Fuel Octobot with 
    oscilattory venting.
    
"""

import e3DPGlobals
import e3DMatrixPrinting
import FancyOctobot2

# SET THESE:                         Hole depth abs machine
hole_insertion_depth_abs_machine = -60.6
local_z_offset_from_machine_abs =  -57.1126 # left_eco_zero machine
module_top_print_height_above_hole_bottom = 2
# Calculate LOCAL coordinate print heights such that
# local abs = abs machine - local_z_offset_from_machine_abs
module_hole_depth = hole_insertion_depth_abs_machine - local_z_offset_from_machine_abs

#module_bottom_layer_thickness = 2
#RTruby, 2014.09.11: mold_hole_depth changed from -3.0 to -2.0 for Experiment C-95
#RTruby, 2014.09.12: mold_hole_depth changed from -2.0 to -1.9 for Experiment C-95, Spider #4
#mold_hole_depth = -2 # FancyOctobot.mold_top - FancyOctobot.mold_depth + module_bottom_layer_thickness

# USING ABS MACHINE COORDS  

# SET THESE: MOLD CORNER COORDINATES - THE ABSOLUTE MACHINE COORDINATES OF THE NOZZLE'S STARTING POSITION AT THE TOP LEFT CORNER OF THE MOLD
mold_home_pos_machine = (445.1098, 146.925) # From August 2014, 444.836, 143.897 - Note by RTruby, 2014.09.10

# SET THESE: HOLE COORDINATES _ ABSOLUTE MACHINE COORDINATES OF THE MODULE HOLES
front_left_hole_pos_machine = (497.751, 114.839)
front_right_hole_pos_machine = (503.476, 114.839)
back_left_hole_pos_machine   = (497.845, 110.902)
back_right_hole_pos_machine = (503.852, 111.023)
check_valve_left_hole_pos_machine = (499.119, 107.452)  # added 20150421 for D-90
check_valve_right_hole_pos_machine = (502.8505, 107.452) # added 20150421 for D-90

#subtract lists element-wise
def subtract(a,b):
    return [a_item - b_item for a_item,b_item in zip(a,b)]
front_left_hole_pos =  subtract(front_left_hole_pos_machine,mold_home_pos_machine) #(496.723 -mold_x, 103.92 -mold_y) # (53.15, -40.26)
front_right_hole_pos = subtract(front_right_hole_pos_machine,mold_home_pos_machine) #(501.100 -mold_x, 103.92 -mold_y) # (57.1, -40.26)
back_left_hole_pos =   subtract(back_left_hole_pos_machine,mold_home_pos_machine) #(497.209 -mold_x, 97.014 -mold_y) # (53, -47.35)
back_right_hole_pos =  subtract(back_right_hole_pos_machine,mold_home_pos_machine) #(500.500 -mold_x, 96.861 -mold_y) # (56.8, -47.35)
check_valve_left_hole_pos = subtract(check_valve_left_hole_pos_machine, mold_home_pos_machine) # added 20150421 for D-90
check_valve_right_hole_pos = subtract(check_valve_right_hole_pos_machine, mold_home_pos_machine) # added 20150421 for D-90

module_front_edge_y = -28.5 # actually, the module front edge is positioned at -29.5, this is just for room
module_front_edge_y = -28 # added 2015.05.07
module_back_edge_y =  -41    # actually, the module back edge is positioned at -39, this is just for room
module_back_edge_y = -47.5     # added 20150421 for D-90, for use with new octobot molds for more complicated modules
hole_flush_dwell_time = 4.0 #was 5.0

line_offset_x = None
centerline_x = None
module_top_print_height = None

def init(centerline, offset, print_height):
    global line_offset_x
    global centerline_x
    global module_top_print_height
    line_offset_x = offset
    centerline_x = centerline
    module_top_print_height = print_height
    print "\tLogic Module Inited with center " + str(centerline_x) + " and offset " + str(line_offset_x) + " at print height " + str(module_top_print_height) + "."
    
def interface_with_hole(Left, Front, to_cliff, Valve = False):
    if (Valve == 1):
        hole_pos = check_valve_left_hole_pos if Left else check_valve_right_hole_pos
    else:
        hole_pos = ((front_left_hole_pos if Left else front_right_hole_pos) if Front else (back_left_hole_pos if Left else back_right_hole_pos))
    
    # The code below was commented out by RTruby on 2015.01.21; module_top_print_height should be module_hole_depth
    #print "\tInterfacing with hole at " + str(hole_pos) + " at depth " + str(module_top_print_height) + "."
    print "\tInterfacing with hole at " + str(hole_pos) + " at depth " + str(module_hole_depth)
    
    e3DPGlobals.g.abs_move(x=hole_pos[0], y=hole_pos[1]) # move above the hole
    e3DMatrixPrinting.print_mode(print_height_abs=module_hole_depth) # go in the hole, start extrusion
    e3DPGlobals.g.dwell(hole_flush_dwell_time)
    
    # Note by RTruby, 2015.01.21 - module_top_print_height was originally -1
    # in the line below; THIS MUST CHANGE - figure out how to 
    
    #if-else below added for D-90
    if (Valve == 1):
        height_offset_between_lines_to_valves = 1.5
        e3DMatrixPrinting.move_z_abs(height = module_top_print_height-height_offset_between_lines_to_valves, vertical_travel_speed = e3DMatrixPrinting.default_z_drag_speed) # drag up to top of hole   
    else:
        e3DMatrixPrinting.move_z_abs(height = module_top_print_height, vertical_travel_speed = e3DMatrixPrinting.default_z_drag_speed) # drag up to top of hole   

    
    if (to_cliff):
        e3DPGlobals.g.feed(e3DMatrixPrinting.default_print_speed)
        e3DPGlobals.g.abs_move(x= (-1 if Left else 1) * line_offset_x +centerline_x, y = (module_front_edge_y if Front else module_back_edge_y)) # move above the hole
    #else:
        #e3DMatrixPrinting.travel_mode()
        