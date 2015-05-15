# -*- coding: utf-8 -*-
"""

Filename: MultiMaterial.py
Author: Ryan L. Truby
Affiliation: Lewis Research Group, Harvard University
Data: 2015.01.19

Description:
    This library contains mecode functions for handling multiple printheads. 
    This code is an adaptation of Dan Fitzgerald's script "MultiMaterial.py". 
    
"""

# Line underneath commented out by RTruby on 2014.09.10 during Experiment C-95
#from aerotech_automation import AerotechAutomator

# Lines underneath added by RTruby on 2014.09.10 during Experiment C-95 during Experiment C-95
from mecode import G
from aerotech_automator import AerotechAutomator

import e3DMatrixPrinting
import e3DPGlobals
import os

cur_filepath = os.path.dirname(os.path.realpath(__file__))
print "FILE PATH IS " + str(cur_filepath)+"."
splitted = cur_filepath.split('/')
n=len(splitted)
cur_dir = ''.join((dir+'/') for dir in splitted[:n-1])
#------------------------------------------------------------------------------
# RTruby, 2014.09.10 - Code below commented out during Experiment C-95:
#
#importFileDir = cur_dir+"mecode\\automation_values.json"
#
# Code below added when previous code was commented out:
importFileDir = cur_dir+"automation_values.json"
#------------------------------------------------------------------------------
print "Importing nozzle offsets from file " + str(importFileDir)

automator = AerotechAutomator()
automator.load_state(importFileDir)

Ax_groove, Ay_groove, zA_granite = automator.home_positions['A']
Bx_groove, By_groove, zB_granite = automator.home_positions['B']

x_grooves = [Ax_groove, Bx_groove]
y_grooves = [Ay_groove, By_groove]
z_granites = [zA_granite, zB_granite]


print "Nozzle A home at (" + str(Ax_groove) + ", " + str(Ay_groove) + ", " + str(zA_granite) + ")"
print "Nozzle B home at (" + str(Bx_groove) + ", " + str(By_groove) + ", " + str(zB_granite) + ")"

tool_axis = (["A","B","C","D"] )#if e3De3DPGlobals.Aerotech else ["z","z","z","z"])
line_pressures = [85,33,87,88]
com_ports = [1, 4, 9, 9] 

cur_tool_index = 0
cur_tool = tool_axis[cur_tool_index]
cur_pressure=line_pressures[cur_tool_index]
cur_com_port = com_ports[cur_tool_index]

def set_cur_tool():
    """"Helper funciton. Sets the current axis letter, pressure, and com port to those corresponding the the current tool index."""
    
    global cur_tool
    global cur_pressure
    global cur_com_port
    
    e3DMatrixPrinting.turn_pressure_off()
    cur_tool = tool_axis[cur_tool_index]
#    cur_pressure=line_pressures[cur_tool_index]
    cur_com_port = com_ports[cur_tool_index]
    
#    e3DPGlobals.g.set_pressure(cur_com_port, cur_pressure)

def change_tool(to_tool_index):
    """Change the current nozzle and default z axis."""
    
    if (cur_tool_index != to_tool_index):
        global cur_tool_index
        global cur_tool
        old_tool = cur_tool
        cur_tool_index = to_tool_index
        set_cur_tool()
        
        e3DPGlobals.g.write("\n; Change Tools from " + old_tool + " to " + tool_axis[cur_tool_index] + ".")
        
        e3DPGlobals.g.rename_axis(z=cur_tool)
        print "Renaming Z axis to " + cur_tool+"."

        if (to_tool_index == 0):
            e3DPGlobals.g.write("G53 ; clear current fixture offset to use Nozzle A (default and homes with position offset)\n")
        else:    
            str_new_nozzle_offset_from_A_x = str(x_grooves[cur_tool_index] - Ax_groove) #"($"+cur_tool+"x-$Ax-($"+cur_tool+"x_dif-$Ax_dif))" #"($Ax_diff-$"+cur_tool+"x_diff)"
            str_new_nozzle_offset_from_A_y = str(y_grooves[cur_tool_index] - Ay_groove) #"($Ay-$"+cur_tool+"y+($Ay_dif-$"+cur_tool+"y_dif))" #"($Ay_diff-$"+cur_tool+"y_diff)"
            str_new_nozzle_offset_from_A_z = str(z_granites[cur_tool_index] - zA_granite) #"($zMeasureA-$zMeasure"+cur_tool+")"  #+($Az_diff-$"+cur_tool+"z_diff))" #"($Az_diff-$"+cur_tool+"z_diff)"
        
            e3DPGlobals.g.write("G53 ; clear any current fixture offset\nG54 X" + str_new_nozzle_offset_from_A_x + " Y" + str_new_nozzle_offset_from_A_y + " " + cur_tool + str_new_nozzle_offset_from_A_z + " ; set a new fixture offset to compensate for the new tools offset.\n")
            #    e3DPGlobals.g.write("G1 X(($" + cur_tool + "x-$" + from_tool + "x-($" + cur_tool + "x_dif-$" + from_tool + "x_dif))) F" + str(e3DMatrixPrinting.default_air_travel_speed))
            #    e3DPGlobals.g.write("G1 Y(($" + from_tool + "y-$" + cur_tool + "y+($" + from_tool + "y_dif-$" + cur_tool + "y_dif)))")

    else:
        print "ERROR: Switched to tool currently in use!"
