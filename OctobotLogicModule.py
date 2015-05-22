# -*- coding: utf-8 -*-
"""

Filename: OctobotLogicModule.py
Author: Ryan L. Truby
Affiliation: Lewis Research Group, Harvard University
Date: 2015.01.19

Description:
    This library contains mecode functions for connecting logic modules with 
    printed microfluidics. 
    This code is an adaptation of Dan Fitzgerald's script "SpiderLogicModule.py"
       
2015.05.15:
    This file copied and pasted to "e3DOctoPv2" to code up a Fuel Octobot with 
    oscilattory venting.
    
"""

# -*- coding: utf-8 -*-
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
   
def get_pressure_channel_back_y():
    #if (LogicModule.module_top_print_height > FancyOctobot2.control_line_height_abs):
    #    ramp_angle = 50
    #    ramp_height = LogicModule.module_top_print_height - FancyOctobot2.control_line_height_abs
    #    ramp_length = ramp_height/np.tan(np.deg2rad(ramp_angle))
    #    print "Ramp height is "+str(ramp_height)+" and length is " + str(ramp_length)
    #    return LogicModule.module_front_edge_y + ramp_length
    #else:
    return LogicModule.module_front_edge_y

pressure_chamber_speed = 0.5
pressure_chamber_connection_dwell_time = 1.0
pressure_chamber_connection_dwell_time = 2.0 # changed on 2015.05.07
pressure_channel_overlap = 0.5 # overlap for pressure chamber connections
pressure_chamber_total_length = 2.5 # was 3, changed on 2015.01.21
pressure_chamber_total_length = 2 # changed on 2015.05.07

# print the front left hole to control line A
def print_output_hole_to_flow_line_with_pressureChamber(Left, hole_pos):
    x_offset_mult = (-1 if Left else 1)

    print "Printing module output to robot channels input via pressure chamber."

    # ramp down down to channel (if needed) will determine min pressure channel back end y
    pressure_channel_back_y = get_pressure_channel_back_y()

    
    # Print a pressure chamber/connection from cliff base to the flow channel ends
    e3DPGlobals.g.write("\n; PRINT PRESSURE CHAMBER FOR " + ("LEFT" if Left else "RIGHT") + " SIDE")
    MultiMaterial.change_tool(1) # switch to platinum
    e3DPGlobals.g.abs_move(x=FancyOctobot2.mold_center_x+x_offset_mult*FancyOctobot2.control_line_connector_x_dist_from_center_line, y=pressure_channel_back_y) # move over front of the front edge of the module ("cliff")
    e3DMatrixPrinting.print_mode(print_height_abs=FancyOctobot2.control_line_height_abs)
    e3DPGlobals.g.dwell(pressure_chamber_connection_dwell_time)
    e3DPGlobals.g.feed(pressure_chamber_speed)
    e3DPGlobals.g.abs_move(x=FancyOctobot2.mold_center_x+x_offset_mult*FancyOctobot2.control_line_connector_x_dist_from_center_line, y=pressure_channel_back_y+pressure_chamber_total_length) # print up to the channel line back end # FancyOctobot2.control_line_back_y + pressure_channel_overlap
    e3DPGlobals.g.dwell(pressure_chamber_connection_dwell_time)
    e3DMatrixPrinting.travel_mode()
    MultiMaterial.change_tool(0)
    
    print "PRESSURE CHAMBER TOTAL LENGTH IS " + str(FancyOctobot2.control_line_back_y + pressure_channel_overlap-pressure_channel_back_y)   
                                                      
    # Print a vertical line in the hole to the top of the module
    e3DPGlobals.g.write("\n; PRINT IN HOLE OF " + ("LEFT" if Left else "RIGHT") + " SIDE")
    LogicModule.interface_with_hole(Left=Left, Front=True, to_cliff = False)
            
    # drag up the cliff and over to the hole top in pluronic
    if (LogicModule.module_top_print_height > FancyOctobot2.control_line_height_abs): 
        print "\t CLIFF NEEDED"
        
        # make a standard cliff - no ramp

        e3DMatrixPrinting.travel_mode()

        ramp_height = 0.0
        e3DPGlobals.g.abs_move(x=FancyOctobot2.mold_center_x+x_offset_mult*FancyOctobot2.control_line_connector_x_dist_from_center_line, y=pressure_channel_back_y+pressure_channel_overlap)
        e3DMatrixPrinting.print_mode(print_height_abs = FancyOctobot2.control_line_height_abs, print_speed = e3DMatrixPrinting.default_inlet_print_speed)
        e3DPGlobals.g.dwell(pressure_chamber_connection_dwell_time)
        e3DPGlobals.g.abs_move(y=pressure_channel_back_y)
        e3DMatrixPrinting.move_z_abs(height = LogicModule.module_top_print_height+ramp_height, vertical_travel_speed = e3DMatrixPrinting.default_z_drag_speed) # drag z up the cliff 
        e3DPGlobals.g.feed(e3DMatrixPrinting.default_print_speed)
        hole_pos = (LogicModule.front_left_hole_pos if Left else LogicModule.front_right_hole_pos)
        e3DPGlobals.g.abs_move(x=hole_pos[0], y=hole_pos[1], z=LogicModule.module_top_print_height) # move above the hole
        e3DPGlobals.g.dwell(pressure_chamber_connection_dwell_time)
        e3DMatrixPrinting.travel_mode()

        ## ramp down from logic module print height to pressure channel (control line) height
        #e3DMatrixPrinting.move_z_abs(height = FancyOctobot2.control_line_height_abs, vertical_travel_speed = e3DMatrixPrinting.default_z_drag_speed) # drag z up the cliff 
        #e3DPGlobals.g.feed(e3DMatrixPrinting.default_print_speed)
        #e3DPGlobals.g.abs_move(x=FancyOctobot2.mold_center_x+x_offset_mult*FancyOctobot2.control_line_connector_x_dist_from_center_line, y=LogicModule.module_front_edge_y)
        #e3DPGlobals.g.abs_move(x=FancyOctobot2.mold_center_x+x_offset_mult*FancyOctobot2.control_line_connector_x_dist_from_center_line, y=pressure_channel_back_y, z=FancyOctobot2.control_line_height_abs) # move over front of the front edge of the module ("cliff")
        #e3DPGlobals.g.abs_move(x=FancyOctobot2.mold_center_x+x_offset_mult*FancyOctobot2.control_line_connector_x_dist_from_center_line, y=pressure_channel_back_y+pressure_channel_overlap) # move over front of the front edge of the module ("cliff")
        #e3DPGlobals.g.dwell(pressure_chamber_connection_dwell_time)
        #e3DMatrixPrinting.travel_mode()
    else:
        print "CLIFF FROM MODULE TO CONTROL lINES NEEDED" 
        print "ERROR: Cliff from pressure lines up tp logic module print height not implmented yet!"
        e3DPGlobals.g.write("ERROR: Cliff from pressure lines up tp logic module print height not implmented yet!")
      
def hook_module_into_fancy_spider():
    print_output_hole_to_flow_line_with_pressureChamber(Left=True, hole_pos = LogicModule.front_left_hole_pos)
    print_output_hole_to_flow_line_with_pressureChamber(Left=False, hole_pos = LogicModule.front_right_hole_pos)

# print_pressureChamber_for_test_octobot is for use with TestOctobot.py
def print_pressureChamber_for_test_octobot(Left, fuel_line, needle_insertion, spacer):
    x_offset_mult = (-1 if Left else 1)

    print "Printing module output to robot channels input via pressure chamber."

    # ramp down down to channel (if needed) will determine min pressure channel back end y
    pressure_channel_back_y = get_pressure_channel_back_y()

    
    # Print a pressure chamber/connection from cliff base to the flow channel ends
    e3DPGlobals.g.write("\n; PRINT PRESSURE CHAMBER FOR " + ("LEFT" if Left else "RIGHT") + " SIDE")
    MultiMaterial.change_tool(1) # switch to platinum
    e3DPGlobals.g.abs_move(x=FancyOctobot2.mold_center_x+x_offset_mult*FancyOctobot2.control_line_connector_x_dist_from_center_line, y=pressure_channel_back_y) # move over front of the front edge of the module ("cliff")
    e3DMatrixPrinting.print_mode(print_height_abs=FancyOctobot2.control_line_height_abs)
    e3DPGlobals.g.dwell(pressure_chamber_connection_dwell_time)
    e3DPGlobals.g.feed(pressure_chamber_speed)
    e3DPGlobals.g.abs_move(x=FancyOctobot2.mold_center_x+x_offset_mult*FancyOctobot2.control_line_connector_x_dist_from_center_line, y=pressure_channel_back_y+pressure_chamber_total_length) # print up to the channel line back end # FancyOctobot2.control_line_back_y + pressure_channel_overlap
    e3DPGlobals.g.dwell(pressure_chamber_connection_dwell_time)
    e3DMatrixPrinting.travel_mode()
    MultiMaterial.change_tool(0) # switch to Pluronic
    # move back to upstream end of reaction chamber
    e3DPGlobals.g.abs_move(x=FancyOctobot2.mold_center_x+x_offset_mult*FancyOctobot2.control_line_connector_x_dist_from_center_line, y=pressure_channel_back_y)
    e3DMatrixPrinting.print_mode(print_height_abs=FancyOctobot2.control_line_height_abs)
    e3DPGlobals.g.dwell(pressure_chamber_connection_dwell_time)
    e3DPGlobals.g.feed(e3DMatrixPrinting.default_print_speed)
    e3DPGlobals.g.abs_move(x=FancyOctobot2.mold_center_x+x_offset_mult*FancyOctobot2.control_line_connector_x_dist_from_center_line+x_offset_mult*spacer,y = pressure_channel_back_y-fuel_line)
    e3DPGlobals.g.feed(e3DMatrixPrinting.default_print_speed/4)
    e3DPGlobals.g.abs_move(y = pressure_channel_back_y-fuel_line-needle_insertion)
    e3DPGlobals.g.dwell(20)
    e3DMatrixPrinting.travel_mode()
    
def print_reaction_chamber_for_osc_venting(Left):
    x_offset_mult = (-1 if Left else 1)

    print "Printing module output to robot channels input via pressure chamber."

    # ramp down down to channel (if needed) will determine min pressure channel back end y
    pressure_channel_back_y = get_pressure_channel_back_y()
    ramp_height = 0.0
    
    # Print a pressure chamber/connection from cliff base to the flow channel ends
    e3DPGlobals.g.write("\n; PRINT PRESSURE CHAMBER FOR " + ("LEFT" if Left else "RIGHT") + " SIDE")
    MultiMaterial.change_tool(1) # switch to platinum
    e3DPGlobals.g.abs_move(x=FancyOctobot2.mold_center_x+x_offset_mult*FancyOctobot2.control_line_connector_x_dist_from_center_line, y=pressure_channel_back_y) # move over front of the front edge of the module ("cliff")
    e3DMatrixPrinting.print_mode(print_height_abs=FancyOctobot2.control_line_height_abs)
    e3DPGlobals.g.dwell(pressure_chamber_connection_dwell_time)
    e3DPGlobals.g.feed(pressure_chamber_speed)
    e3DPGlobals.g.abs_move(x=FancyOctobot2.mold_center_x+x_offset_mult*FancyOctobot2.control_line_connector_x_dist_from_center_line, y=pressure_channel_back_y+pressure_chamber_total_length) # print up to the channel line back end # FancyOctobot2.control_line_back_y + pressure_channel_overlap
    e3DPGlobals.g.dwell(pressure_chamber_connection_dwell_time)
    e3DMatrixPrinting.travel_mode()
    MultiMaterial.change_tool(0)
    
    e3DPGlobals.g.abs_move(x=FancyOctobot2.mold_center_x+x_offset_mult*FancyOctobot2.control_line_connector_x_dist_from_center_line, y=pressure_channel_back_y+pressure_channel_overlap)
    e3DMatrixPrinting.print_mode(print_height_abs = FancyOctobot2.control_line_height_abs, print_speed = e3DMatrixPrinting.default_inlet_print_speed)
    e3DPGlobals.g.dwell(pressure_chamber_connection_dwell_time)
    e3DPGlobals.g.abs_move(y=pressure_channel_back_y)
    e3DMatrixPrinting.move_z_abs(height = LogicModule.module_top_print_height+ramp_height, vertical_travel_speed = e3DMatrixPrinting.default_z_drag_speed) # drag z up the cliff 
    e3DMatrixPrinting.travel_mode()
    
    print "PRESSURE CHAMBER TOTAL LENGTH IS " + str(FancyOctobot2.control_line_back_y + pressure_channel_overlap-pressure_channel_back_y)  
    
def print_both_reaction_chambers_for_osc_venting():
    print_reaction_chamber_for_osc_venting(Left = True)
    print_reaction_chamber_for_osc_venting(Left = False)
    
    