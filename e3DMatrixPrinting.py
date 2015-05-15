# -*- coding: utf-8 -*-
"""

Filename: e3DMatrixPrinting.py
Author: Ryan L. Truby
Affiliation: Lewis Research Group, Harvard University
Data: 2015.01.19

Description:
    This library contains mecode functions for handling multiple printheads. 
    This code is an adaptation of Dan Fitzgerald's script "MultiMaterial.py". 
    
"""

# -*- coding: utf-8 -*-

import numpy as np

from mecode import G
import e3DPGlobals
import MultiMaterial
import e3DMatrixPrinting

# DEFAULT PRINTING PARAMETERS - feeds and speeds (All speeds in mm/s)
default_start_stop_dwell_time = 0.2 # ink hysteresis compensation. Dwell for this long after starting extrusion but before moving, and after stopping but before stopping extrusion
default_air_travel_speed = 20 # travel speed in air (movement above the mold)
default_matrix_travel_speed = 0.5 # speed to travel in matrix (used for traveling vertically)
default_inlet_length = 2 # length of needle insertion inlets
default_inlet_print_speed = 0.5 # speed for making needle insertion inlets
default_print_speed = 1.5 # print speed used for most channels/traces
default_mold_z_zero = 1 # Height of the top of the mold relative to work z zero. Unless otherwise specified, all other z coordinates or heights are relative to the work zero (top of ecoflex), NOT the mold top.
default_mold_z_zero = -30.844 # for not printing in mold
default_travel_height_abs = default_mold_z_zero + 3 # height above the work zero (ecoflex top) to travel in air
# default_z_drag_speed set to 0.25 on 2014.11.21 by RLT; was originally 0.5
default_z_drag_speed = 0.5/4 # speed to "drag" vertical connections up
default_z_drag_speed = 0.5/2 # added 2015.02.06


# Pressure control Macros
pressure_on = False
def turn_pressure_off(com_port = -1, start_stop_dwell_time = default_start_stop_dwell_time):
    """If the current nozzle's (MultiMaterial.cur_tool) pressure is not on, turns it on"""
    
    global pressure_on
    if (pressure_on):
        #get around the inability to use global vars as default arg values
        if (com_port == -1):
            com_port=MultiMaterial.cur_com_port
        e3DPGlobals.g.write("; Toggle pressure on com port " + str(com_port) + " to turn tool " + MultiMaterial.cur_tool + " off.")
        
        e3DPGlobals.g.toggle_pressure(com_port)
        e3DPGlobals.g.dwell(start_stop_dwell_time)
        pressure_on = False

def turn_pressure_on(com_port = -1, start_stop_dwell_time = default_start_stop_dwell_time):
    """If the current nozzle's (MultiMaterial.cur_tool) pressure is on, turns it off"""
        
    global pressure_on
    if (not pressure_on):
        #get around the inability to use global vars as default arg values
        if (com_port == -1):
            com_port=MultiMaterial.cur_com_port
        e3DPGlobals.g.write("; Toggle pressure on com port " + str(com_port) + " to turn tool " + MultiMaterial.cur_tool + " on.")
        
        e3DPGlobals.g.dwell(start_stop_dwell_time)
        e3DPGlobals.g.toggle_pressure(com_port)
        pressure_on = True

def move_z_abs(height, vertical_travel_speed = default_matrix_travel_speed): #z_axis = "DEFAULT",
    """Move the given z axis in absolute work coordinates"""
            
    #TODO: end on the same print speed we began on
    #maybe move slower while under mold_z_abs
    last_pos = (e3DPGlobals.g.position_history[-1] if len(e3DPGlobals.g.position_history)>0 else None)
    last_z = (last_pos[2] if (last_pos is not None) else -np.inf)
    #print "move_abs_z: Last Pos is " + str(last_pos) + " last z is " + str(last_z) + "."

    if (height-last_z != 0):
        e3DPGlobals.g.feed(vertical_travel_speed)
        e3DPGlobals.g.abs_move(z=height)
        last_z=height
    else:
        print "WARNING: abs z move of 0 length!"
    
def travel_mode(travel_speed = default_air_travel_speed, travel_height_abs = default_travel_height_abs):
    """"Stop Extrusion, move to travel height"""
    
    e3DPGlobals.g.write("\n; Enter Travel Mode to height " + str(travel_height_abs) + " where mold top is " + str(default_mold_z_zero)) 
   
    last_pos = (e3DPGlobals.g.position_history[-1] if len(e3DPGlobals.g.position_history)>0 else None)
    last_z = (last_pos[2] if (last_pos is not None) else -np.inf)
#    print "TRAVEL_MODE: last_pos was " + str(last_pos) + " last_z was " + str(last_z)
    turn_pressure_off()
    if (last_z < default_mold_z_zero):
        move_z_abs(default_mold_z_zero, vertical_travel_speed=e3DMatrixPrinting.default_matrix_travel_speed)
    move_z_abs(travel_height_abs, vertical_travel_speed = travel_speed)  
    
    #    print "Travel mode to height " + str((last_pos[2] if (last_pos) else np.inf))
    e3DPGlobals.g.write("; Now in travel mode.") 
    
def print_mode(print_height_abs, travel_speed = default_matrix_travel_speed, print_speed = default_print_speed):
    """Move to print height, start Extrusion"""
    
    e3DPGlobals.g.write("\n; Enter Print Mode to print height " + str(print_height_abs) + " where mold top is " + str(default_mold_z_zero)) 
    
    # go down to to mold zero if we're above it
    last_pos = (e3DPGlobals.g.position_history[-1] if len(e3DPGlobals.g.position_history)>0 else None)
    last_z = (last_pos[2] if (last_pos is not None) else -np.inf)
#    print "PRINT_MODE: last_pos was " + str(last_pos) + " last_z was " + str(last_z)
    if (last_z > default_mold_z_zero):
        move_z_abs(default_mold_z_zero, vertical_travel_speed=default_air_travel_speed)
        
    # go the rest of the way in the default_matrix_travel_speed
    if (print_height_abs<default_mold_z_zero):
        move_z_abs(print_height_abs,  vertical_travel_speed=travel_speed)
    else:
        print "e3DMatrixPrinting - print_mode() ERROR: printmode print_height_abs " + str(print_height_abs) + " is above the mold_z_zero " + str(default_mold_z_zero) + "!"
    turn_pressure_on()
    e3DPGlobals.g.feed(print_speed)
    e3DPGlobals.g.write("; Now in print mode.") 
    
def move_x(distance, theta=0):
    global g
    if (distance==0):
        print "e3DMatrixPrinting - move_y WARNING:No y movement!"
    e3DPGlobals.g.move(x=np.cos(theta)*distance, y=np.sin(theta)*distance)        
     
def move_y(distance, theta=0):
    global g
    if (distance==0):
        print "e3DMatrixPrinting - move_x WARNING:No x movement!"
    e3DPGlobals.g.move(x=-np.sin(theta)*distance, y=np.cos(theta)*distance)                     
 
def move_xy(x_distance, y_distance, theta=0):
    global g
    C=np.cos(theta)
    S=np.sin(theta)
    if (x_distance==0 and y_distance==0):
        print "e3DMatrixPrinting - move_xy WARNING:No x or y movement!"
    e3DPGlobals.g.move(x=x_distance*C-y_distance*S, y=x_distance*S+y_distance*C)