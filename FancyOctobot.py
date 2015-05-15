# -*- coding: utf-8 -*-
"""

Filename: FancyOctobot.py
Author: Ryan L. Truby
Affiliation: Lewis Research Group, Harvard University
Data: 2015.01.19

Description:
    This code prints the downstream components of a "fancy" octobot. These 
    components include the actuators and all networks interconnecting them. 
    Interconnects with a logic module are also printed. This code is an 
    adaptation based off Daniel Fitzgerald's script "FancySpider.py".
    
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
import OctobotLogicModule 

# ------------------------------------------------------------------------------
# ALL MOLD-SPECIFIC PARAMETERS; HANDLE WITH TREMENDOUS CARE

# Parameters related to the mold's top and bottom
mold_top = 0.8        # absolute zero of the top of the mold, also equal to PDMS actuator thickness
e3DMatrixPrinting.default_mold_z_zero = mold_top
e3DMatrixPrinting.default_travel_height_abs = mold_top + 5
mold_depth = 4.34     # total depth of the body of the robot
mold_depth = 8.7      # added 2015.02.03

# Parameters related to the mold's sides and width
mold_center_x = 55    # anteroposterior axis, relative to mold's top left corner
mold_body_width = 2 * (55-49.4)   # width of narrowest part of the robot body

#  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  - 
# Parameters related to actuators and downstream network geometry 
actuator_print_height_offset = 0.1     # nozzle height above EcoFlex
actuator_print_height = 0 + actuator_print_height_offset # print height above EcoFlex
arm_rows_shouder_x_centerline_offsets = [mold_center_x-x for x in [51.64, 50.11, 49.40, 49.01]]
arm_rows_shoulder_y_abs = [-1*y for y in [22.07, 25.13, 28.35, 31.56]] # , 39.56
arm_shoulder_angles_deg = [20, -4.0, -25.0, -58.0]
arm_elbow_angles_deg = [80.0, 70.0, -62.0, -70.0] # first angle was 85.0
# Note from RTruby, 2014.01.20: the first variable should be 0.0, but the
# program will not let me put a non-zero arm_upperarm_length for the first
# actuator. I'll leave 0.001 here for now.
arm_upperarm_lengths = [0.001, 10.73, 10.65, 10.76]
arm_forarm_preActuator_length = 1
arm_forarm_preActuator_length_arm1 = 4
arm_forarm_preActuator_length_arm1 = 12
arm_forarm_postActuator_lengths = [3.0, 5.0, 4.0, 6.0]
# Note from RTruby, 2014.01.19: make sure to understand the following variables 
arm_elbow_angles_deg_relative = list(np.array(arm_shoulder_angles_deg)-np.array(arm_elbow_angles_deg))
routing_branchpoint_spacing = 1 # distance between routing branches
routing_turnpoint_from_lines_x = 0.75 # distance from the control lines that the routing lines turn towards thier legs
routing_front_y = -19.5
routing_front_y = -19.5 - 2 #added 2015.01.21
routing_front_y = -19.5 # added 2015.01.23
routing_branchpoints_y = [routing_front_y - n*routing_branchpoint_spacing for n in range(6)] # len(arm_rows_shoulder_y_abs)
routing_back_y = routing_branchpoints_y[-1]-routing_branchpoint_spacing
routing_leg_offshoot_points = [routing_front_y-(routing_front_y - routing_back_y)*(n/4.0) for n in range(4)]
print "Routing branchpoints ys are " + str(routing_branchpoints_y)
print "Routing leg offshoot ys are " + str(routing_leg_offshoot_points)

# Parameters related to control lines
control_line_bridge_pass = 1
control_line_height_abs = mold_top - mold_depth/2.0 - 0.5*control_line_bridge_pass# height of control line channels A and B - transverse midplane of robot
control_line_height_abs = -2.5 # added 2015.02.03
print "Control Line print height " + str(control_line_height_abs)
control_line_bridge_height_abs = control_line_height_abs + control_line_bridge_pass# (0+control_line_height_abs)/2.0 # height to bridge over the control lines
control_line_x_dist_from_center_line = 0.75 #(1.0/6.0)*mold_body_width # distance control lines A and B are from the centerline of the robot (to the left and right respectivly)
control_line_A_x = mold_center_x - control_line_x_dist_from_center_line
control_line_B_x = mold_center_x + control_line_x_dist_from_center_line

# Parameters related to control line connections
control_line_back_y = OctobotLogicModule.get_pressure_channel_back_y() + OctobotLogicModule.pressure_chamber_total_length - OctobotLogicModule.pressure_channel_overlap #routing_back_y - 0.5
control_line_connector_x_dist_from_center_line = 1.5 # distance from the pressure lines and the flow connectors they attach to to the centerline of the robot
control_line_connector_length = 0.5
control_line_A_connection_x = mold_center_x - control_line_connector_x_dist_from_center_line
control_line_B_connection_x = mold_center_x + control_line_connector_x_dist_from_center_line
#  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  - 
# ------------------------------------------------------------------------------

# Rows are ordered front-to-back from 0-3 inclusive (4 total)
def print_left_actuator(row, arm_forarm_preActuator_length, num_pads):
    e3DMatrixPrinting.move_z_abs(actuator_print_height)
    Actuators.print_actuator(theta = 0.5*np.pi-np.deg2rad(arm_shoulder_angles_deg[row]), elbow_angle = np.deg2rad(arm_elbow_angles_deg_relative[row]), upperarm_length = arm_upperarm_lengths[row], forarm_preActuator_length=arm_forarm_preActuator_length, forarm_postActuator_length = arm_forarm_postActuator_lengths[row], num_pads = num_pads)

right_side_offset = 5 #if it's not changed from 5 as it shood be then we'll know something's w
def print_right_actuator(row, arm_forarm_preActuator_length, num_pads):
    e3DMatrixPrinting.move_z_abs(actuator_print_height+right_side_offset)
    Actuators.print_actuator(theta = -0.5*np.pi+np.deg2rad(arm_shoulder_angles_deg[row]), elbow_angle = -np.deg2rad(arm_elbow_angles_deg_relative[row]), upperarm_length = arm_upperarm_lengths[row], forarm_preActuator_length=arm_forarm_preActuator_length, forarm_postActuator_length = arm_forarm_postActuator_lengths[row], num_pads = num_pads)
    
def print_robot(ecoflex_zero_left, ecoflex_zero_right, func_print_internal_soft_logic):
    """"Print a fancy robot in the mold. Assume we start at the front left corner of the mold"""

    # PRINT_SPECIFIC PARAMETERS
    MACHINE_ZERO = ecoflex_zero_left # zero on the top of the left ecoflex layer
    MACHINE_ZERO_RIGHT = ecoflex_zero_right # the top of the left ecoflex layer
    global right_side_offset
    right_side_offset = MACHINE_ZERO_RIGHT-MACHINE_ZERO # added to the print height of the right actuators
    
    '''
    Note from RTruby:
        On Jan 19, 2015, I went through all of Dan Fitzgerald's code, and made
        sure I understood anything up until this comment.
        
        GO BACK AND REVIEW THE CODE BELOW AND WHAT IT'S DOING.
    '''     
                                                   
    ################ START PRINTING ################
    
    # set the current X and Y as the origin of the current work coordinates
    e3DPGlobals.g.absolute()     
        
    # go to our absolute work zero (mold top left corner)
    e3DPGlobals.g.write("POSOFFSET CLEAR X Y U A B; clear all position offsets and work coordinates.") #We should start in machine coordinates because the homing routine clears all position offsets, but clear them again anyway just incase
    e3DPGlobals.g.write("; Moving to travel height " + str(e3DMatrixPrinting.default_travel_height_abs) + " above left zero " + str(ecoflex_zero_left) + ".")
    e3DMatrixPrinting.move_z_abs(ecoflex_zero_left + e3DMatrixPrinting.default_travel_height_abs, vertical_travel_speed=e3DMatrixPrinting.default_air_travel_speed) # calculate the absolute default_travel_height relative to our zero and go to it
    
    #MultiMaterial.change_tool(1)
    e3DPGlobals.g.write("; Moving B to Travel Height")
    #e3DPGlobals.g.relative()
    #e3DPGlobals.g.move(x=0.1, y=0.1) # appease the normals
    #e3DPGlobals.g.absolute()
    #B_Height = e3DMatrixPrinting.default_travel_height_abs+0.1
    #e3DMatrixPrinting.move_z_abs(ecoflex_zero_left + B_Height, vertical_travel_speed=e3DMatrixPrinting.default_air_travel_speed) # calculate the absolute default_travel_height relative to our zero and go to it
    #NOTE: Do NOT change tools for this. Then the Nozzle z offsets will be applied twice. (Once with (92, one more with the tool change fixture offet)
    #: TODO: the downside of this is that the nozzles will travel at their unoffset heights (B will be above or below A and may crash if the discrepency is larger than the print travel height)
    e3DPGlobals.g.write("G1 B"+str(ecoflex_zero_left + e3DMatrixPrinting.default_travel_height_abs))
    #MultiMaterial.change_tool(0)

    
    # set this current mold zero as the work zero (set clearany current position offsets
    e3DPGlobals.g.write("\nG92 X0 Y0 "+MultiMaterial.cur_tool+str(e3DMatrixPrinting.default_travel_height_abs)+" " + MultiMaterial.tool_axis[1]+str(e3DMatrixPrinting.default_travel_height_abs)+" ; set the current position as <default_travel_height_abs> above the the absolute work coordinate zero origin")
    e3DPGlobals.g.feed(e3DMatrixPrinting.default_air_travel_speed)
    MultiMaterial.set_cur_tool()
    e3DPGlobals.g.write(" ; READY TO PRINT")

    # Print the soft logic minus the final 'glue' flow lines to the rest of the robot
    # VISITOR PATTERN - Call the logic printing function we were given as an argument. Give it parameters it needs specific to this robot's morphology
    func_print_internal_soft_logic(flow_connector_height_abs = control_line_height_abs, centerline_x = mold_center_x, flow_connectors_centerline_offset = control_line_connector_x_dist_from_center_line, flow_connectors_back_y = control_line_back_y)
    
    
    ################ Control Lines and Actuators ###############   
                                                                 
    #print control line A
    cur_arm_row = 0
    if (e3DPGlobals.g.position_history[-1][0] != control_line_A_connection_x and e3DPGlobals.g.position_history[-1][0] != control_line_back_y):
        e3DPGlobals.g.abs_move(x=control_line_A_connection_x, y = control_line_back_y)
    e3DMatrixPrinting.print_mode(print_height_abs = control_line_height_abs, print_speed = e3DMatrixPrinting.default_inlet_print_speed)
    e3DPGlobals.g.abs_move(y = control_line_back_y+OctobotLogicModule.pressure_channel_overlap)#control_line_connector_length)
    e3DPGlobals.g.feed(e3DMatrixPrinting.default_print_speed)
    if (control_line_A_x != control_line_A_connection_x or routing_back_y != control_line_back_y+control_line_connector_length):
        e3DPGlobals.g.abs_move(x=control_line_A_x, y = routing_back_y)
    e3DPGlobals.g.abs_move(x=control_line_A_x, y = routing_front_y)
            
    # print top right actuator A1 directly from end of control line A
    e3DMatrixPrinting.move_z_abs(control_line_bridge_height_abs)
    e3DPGlobals.g.feed(e3DMatrixPrinting.default_print_speed)
    e3DPGlobals.g.abs_move(x=control_line_A_x - routing_turnpoint_from_lines_x, y=routing_leg_offshoot_points[0])
    e3DPGlobals.g.abs_move(x=mold_center_x-arm_rows_shouder_x_centerline_offsets[cur_arm_row], y=arm_rows_shoulder_y_abs[cur_arm_row])
    print_left_actuator(row=cur_arm_row, arm_forarm_preActuator_length=arm_forarm_preActuator_length_arm1, num_pads = 1)

    #print control line B
    e3DPGlobals.g.abs_move(x=control_line_B_connection_x, y = control_line_back_y)
    e3DMatrixPrinting.print_mode(print_height_abs = control_line_height_abs, print_speed = e3DMatrixPrinting.default_inlet_print_speed)
    e3DPGlobals.g.abs_move(x=control_line_B_connection_x, y = control_line_back_y+control_line_connector_length)
    e3DPGlobals.g.feed(e3DMatrixPrinting.default_print_speed)
    if (control_line_B_x != control_line_B_connection_x or routing_back_y != control_line_back_y+control_line_connector_length):
        e3DPGlobals.g.abs_move(x=control_line_B_x, y = routing_back_y)
    e3DPGlobals.g.abs_move(x=control_line_B_x, y = routing_front_y) # arm_rows_shoulder_y_abs[cur_arm_row]
    
    # print top right actuator B1 directly from end of control line B
    e3DMatrixPrinting.move_z_abs(control_line_bridge_height_abs)
    e3DPGlobals.g.feed(e3DMatrixPrinting.default_print_speed)
    e3DPGlobals.g.abs_move(x=control_line_B_x + routing_turnpoint_from_lines_x, y=routing_leg_offshoot_points[0])
    e3DPGlobals.g.abs_move(x=mold_center_x+arm_rows_shouder_x_centerline_offsets[cur_arm_row], y=arm_rows_shoulder_y_abs[cur_arm_row])
    print_right_actuator(row=cur_arm_row, arm_forarm_preActuator_length=arm_forarm_preActuator_length_arm1, num_pads = 1)
            
    # print actuator A3 (second from top (row 1), right) bridging straight over control line B
    cur_arm_row = 1
    e3DPGlobals.g.abs_move(x=control_line_A_x, y=routing_branchpoints_y[1]) # y=arm_rows_shoulder_y_abs[cur_arm_row]
    e3DMatrixPrinting.print_mode(print_height_abs = control_line_height_abs)
    e3DMatrixPrinting.move_z_abs(control_line_bridge_height_abs)
    e3DPGlobals.g.feed(e3DMatrixPrinting.default_print_speed)
    e3DPGlobals.g.abs_move(x=control_line_B_x)
    e3DPGlobals.g.abs_move(x=control_line_B_x + routing_turnpoint_from_lines_x, y=routing_leg_offshoot_points[1])
    e3DPGlobals.g.abs_move(x=mold_center_x+arm_rows_shouder_x_centerline_offsets[cur_arm_row], y=arm_rows_shoulder_y_abs[cur_arm_row])
    print_right_actuator(row=cur_arm_row, arm_forarm_preActuator_length=arm_forarm_preActuator_length, num_pads = 2)
    
    #print actuator B3 (second from top (row 1), left) going around the control line of A3
    e3DPGlobals.g.abs_move(x=control_line_B_x, y=routing_branchpoints_y[2]) # connect to the control line midway between arm rows 1 and 2
    e3DMatrixPrinting.print_mode(print_height_abs = control_line_height_abs)
    e3DMatrixPrinting.move_z_abs(control_line_bridge_height_abs)
    e3DPGlobals.g.feed(e3DMatrixPrinting.default_print_speed)
    e3DPGlobals.g.abs_move(x=control_line_A_x)
    e3DPGlobals.g.abs_move(x=control_line_A_x - routing_turnpoint_from_lines_x, y=routing_leg_offshoot_points[1])
    e3DPGlobals.g.abs_move(x=mold_center_x-arm_rows_shouder_x_centerline_offsets[cur_arm_row], y=arm_rows_shoulder_y_abs[cur_arm_row])
    print_left_actuator(cur_arm_row, arm_forarm_preActuator_length, num_pads = 2)
    
    ##print actuator A2 (second from bottom (row 3), left)
    cur_arm_row = 2
    e3DPGlobals.g.abs_move(x=control_line_A_x, y = routing_branchpoints_y[3]) # arm_rows_shoulder_y_abs[cur_arm_row]
    e3DMatrixPrinting.print_mode(print_height_abs = control_line_height_abs)
    e3DMatrixPrinting.move_z_abs(control_line_bridge_height_abs)
    e3DPGlobals.g.feed(e3DMatrixPrinting.default_print_speed)
    e3DPGlobals.g.abs_move(x=control_line_A_x - routing_turnpoint_from_lines_x, y=routing_leg_offshoot_points[2])
    e3DPGlobals.g.abs_move(x=mold_center_x-arm_rows_shouder_x_centerline_offsets[cur_arm_row], y=arm_rows_shoulder_y_abs[cur_arm_row])
    print_left_actuator(cur_arm_row, arm_forarm_preActuator_length, num_pads = 2)
    
    #print actuator B2 (second from bottom, right)
    e3DPGlobals.g.abs_move(x=control_line_B_x, y = routing_branchpoints_y[3]) # arm_rows_shoulder_y_abs[cur_arm_row]
    e3DMatrixPrinting.print_mode(print_height_abs = control_line_height_abs)
    e3DMatrixPrinting.move_z_abs(control_line_bridge_height_abs)
    e3DPGlobals.g.feed(e3DMatrixPrinting.default_print_speed)
    e3DPGlobals.g.abs_move(x=control_line_B_x + routing_turnpoint_from_lines_x, y=routing_leg_offshoot_points[2])
    e3DPGlobals.g.abs_move(x=mold_center_x+arm_rows_shouder_x_centerline_offsets[cur_arm_row], y=arm_rows_shoulder_y_abs[cur_arm_row])
    print_right_actuator(cur_arm_row, arm_forarm_preActuator_length, num_pads = 2)
    
    #print actuator B4 (bottom (row 3), left) bridging over control line A
    cur_arm_row=3
    e3DPGlobals.g.abs_move(x=control_line_B_x, y = routing_branchpoints_y[4]) # arm_rows_shoulder_y_abs[cur_arm_row]
    e3DMatrixPrinting.print_mode(print_height_abs = control_line_height_abs)
    e3DMatrixPrinting.move_z_abs(control_line_bridge_height_abs)
    e3DPGlobals.g.feed(e3DMatrixPrinting.default_print_speed)
    e3DPGlobals.g.abs_move(x=control_line_A_x)
    e3DPGlobals.g.abs_move(x=control_line_A_x - routing_turnpoint_from_lines_x, y=routing_leg_offshoot_points[3])
    e3DPGlobals.g.abs_move(x=mold_center_x-arm_rows_shouder_x_centerline_offsets[cur_arm_row], y=arm_rows_shoulder_y_abs[cur_arm_row])
    print_left_actuator(cur_arm_row, arm_forarm_preActuator_length, num_pads = 2)
        
    #print actuator A4 (bottom right) going around the control line of B4
    e3DPGlobals.g.abs_move(x=control_line_A_x, y=routing_branchpoints_y[5]) # y=(arm_rows_shoulder_y_abs[cur_arm_row]+arm_rows_shoulder_y_abs[cur_arm_row-1])/2.0
    e3DMatrixPrinting.print_mode(print_height_abs = control_line_height_abs)
    e3DMatrixPrinting.move_z_abs(control_line_bridge_height_abs)
    e3DPGlobals.g.feed(e3DMatrixPrinting.default_print_speed)
    e3DPGlobals.g.abs_move(x=control_line_B_x)
    e3DPGlobals.g.abs_move(x=control_line_B_x + routing_turnpoint_from_lines_x, y=routing_leg_offshoot_points[3])
    e3DPGlobals.g.abs_move(x=mold_center_x+arm_rows_shouder_x_centerline_offsets[cur_arm_row], y=arm_rows_shoulder_y_abs[cur_arm_row])
    print_right_actuator(cur_arm_row, arm_forarm_preActuator_length, num_pads = 2)

    #go back to home
    e3DPGlobals.g.abs_move(x=0,y=0)
