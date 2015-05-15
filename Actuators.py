# -*- coding: utf-8 -*-
"""
Actuators.py
Daniel Fitzgerald, Harvard Lewis Research Group
07/23/2014

Library of mecode functions for printing soft robot actuators.
"""

import numpy as np

import e3DMatrixPrinting
import e3DPGlobals

def print_actuator(upperarm_length = 11, forarm_preActuator_length = 1, elbow_angle = 0, forarm_postActuator_length =4, theta=0, num_pads = 2):
    """Prints a soft actuator with the stem starting in the current position and rotated by theta. Assume nozzle is already at the correct height and already in print mode. (Continuation feature)
    
    Basic actuator geometry is
    
    y
    ^
    |
    +-->x
    
    []  second pad
    |   forarm_length
    []  first pad
    |   forarm_preActuator_length
    .   elbow (with optional bend angle of the forarm)
    |   upperarm_length
    x   start point

    """
                                                   
    # pad parameters
    pad_length = 2.6 # length in Y
    pad_width = 2.6 # width in x
    n_meanders = 8
    pad_print_speed = 4.5
    pad_print_speed = pad_print_speed * 0.75
    meander_separation_dist = pad_length/n_meanders
    vent_stem_length = 1 # added 20150327
    vent_spot_dwell = 0 # added 20150327
    
    def print_actuator_pad():
        """Helper funciton. Print one actuator pad here"""
        e3DPGlobals.g.write("\n; Print actuator pad.")
        e3DPGlobals.g.feed(pad_print_speed)
        e3DMatrixPrinting.move_x(-pad_width/2, theta) #move to the lower left corner of the pad
        for meander in range(n_meanders-1):
            e3DMatrixPrinting.move_xy(x_distance=pad_width, y_distance=meander_separation_dist,theta=theta)        # horizontal across the whole pad
            e3DMatrixPrinting.move_x(-pad_width,theta)
        e3DMatrixPrinting.move_xy(x_distance=pad_width, y_distance=meander_separation_dist,theta=theta)    
        e3DMatrixPrinting.move_x(-pad_width/2, theta)           # move to the middle of the top of the pad
    
    e3DPGlobals.g.write("\n; PRINT ACTUATOR.")                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               
    e3DPGlobals.g.relative()                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   
    e3DPGlobals.g.feed(e3DMatrixPrinting.default_print_speed*3) # *3 added on D-52
    e3DPGlobals.g.feed(e3DMatrixPrinting.default_print_speed/2) # *3 added on D-52
    e3DPGlobals.g.feed(e3DMatrixPrinting.default_print_speed) 
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            
    #print the forarm up to the elbow bend
    e3DMatrixPrinting.move_y(upperarm_length, theta)
    
    theta+=elbow_angle # make a turn at the elbow
    
    #print the forarm after the elbow bend to the first pad
    e3DMatrixPrinting.move_y(forarm_preActuator_length, theta)
    
    # block below added on 2015.04.03
    if num_pads == 1:
        sign = (-1 if theta<0 else 1)
        theta = sign * 120
        print "theta_new"
        print theta
        e3DMatrixPrinting.move_y(1, theta) 
    
    #print actuator pad 1
    print_actuator_pad()
    
    if num_pads == 2:
        #print connection stem to actuator second actuator pad
        e3DPGlobals.g.feed(e3DMatrixPrinting.default_print_speed*3) # *3 added on D-52
        e3DPGlobals.g.feed(e3DMatrixPrinting.default_print_speed/2) # *3 added on D-52
        e3DPGlobals.g.feed(e3DMatrixPrinting.default_print_speed) 
        e3DMatrixPrinting.move_y(forarm_postActuator_length, theta)
    
        #print actuator pad 2
        print_actuator_pad()
    
    #added 20150327: print short stem to vent
    e3DMatrixPrinting.move_y(vent_stem_length, theta)
    e3DPGlobals.g.dwell(vent_spot_dwell)
    #added 20150501, Experiment D-104 to avoid blobs at end of actuator
    e3DMatrixPrinting.turn_pressure_off(com_port = 1, start_stop_dwell_time = 0)
    e3DMatrixPrinting.move_y(2, theta)
       
    e3DPGlobals.g.absolute()                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 
    e3DMatrixPrinting.travel_mode()
    e3DPGlobals.g.write("\n; Done with Actuator.\n\n")