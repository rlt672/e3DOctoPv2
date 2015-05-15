# -*- coding: utf-8 -*-
"""
e3DPDemo.py
Ryan Truby, Harvard Lewis Research Group
04/16/2015

Library of mecode functions for printing soft robot pneunet actuators.
"""

import numpy as np
import e3DMatrixPrinting
import e3DPGlobals

# Define the export file that will hold the generated G code
import os
cur_filepath = os.path.dirname(os.path.realpath(__file__))
splitted = cur_filepath.split('/')
n = len(splitted)
cur_dir = ''.join((dir+'/') for dir in splitted[:n-1])
exportFileDir = cur_dir + "soft_robotics_octobot/e3DPDemo.pgm"
print "Exporting to file " + str(exportFileDir)

e3DPGlobals.init_G(exportFileDir)

def print_triangular_meander(print_height, N, edge, angle):
    
    n = np.sqrt(N)
    m = n
    n = n - 1
    pi = 3.141592653589793
    angle_converted = pi*angle/180.0
    h = edge*(np.sqrt(3.0)/2)
    
    e3DMatrixPrinting.print_mode(print_height_abs = print_height, print_speed = 1.5)
    e3DPGlobals.g.relative()
    
    def trace_inner_units(repeats):
        for repeat in range(int(repeats)):
            e3DMatrixPrinting.move_xy(x_distance = edge/2, y_distance = h, theta = angle_converted)
            e3DMatrixPrinting.move_xy(x_distance = edge/2, y_distance = -1*h, theta = angle_converted)

    def trace_to_next_level(num_edges):
        e3DMatrixPrinting.move_x(distance = -1*num_edges*edge, theta = angle_converted)
    
    def trace_outer_edges(num_edges):
        #e3DMatrixPrinting.move_xy(x_distance = edge/2, y_distance = h, theta = pi/3+angle_converted)
        e3DMatrixPrinting.move_xy(x_distance = num_edges*edge/2, y_distance = -1*num_edges*h, theta = angle_converted)
        e3DMatrixPrinting.move_xy(x_distance = -1*num_edges*edge, y_distance = 0, theta = angle_converted)
    
    while (n != 0.0):
        trace_inner_units(repeats = n)
        e3DMatrixPrinting.move_x(distance = edge, theta = pi/3+angle_converted)
        trace_to_next_level(num_edges = n)
        n = n - 1
    e3DMatrixPrinting.move_xy(x_distance = edge/2, y_distance = h, theta = angle_converted)
    trace_outer_edges(num_edges = m)

number_of_edge_triangles = 5
print_triangular_meander(-79.7, np.power(number_of_edge_triangles, 2), 8.0, 0.0)
e3DMatrixPrinting.travel_mode()
h = 15.0*(np.sqrt(3.0)/2)
e3DMatrixPrinting.move_y(h*number_of_edge_triangles/3)
number_of_edge_triangles = 8
print_triangular_meander(-77.7, np.power(number_of_edge_triangles, 2), 5.0, -60.0)
e3DMatrixPrinting.travel_mode()

e3DPGlobals.g.view('mayavi')
#e3DPGlobals.g.view()
e3DPGlobals.g.teardown()
