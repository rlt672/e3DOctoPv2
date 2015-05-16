# -*- coding: utf-8 -*-

"""

Filename: e3DPGlobals.py
Author: Ryan L. Truby
Affiliation: Lewis Research Group, Harvard University
Date: 2015.01.19

Description:
    This library contains mecode functions and variables used globally by other
    mecode libraries as well as other scripts for embedded 3D (e3D) printing. 
    This code is an adaptation of Dan Fitzgerald's original script, 
    "PrintingGlobals.py", written on 2014.07.23.
      
2015.05.15:
    This file copied and pasted to "e3DOctoPv2" to code up a Fuel Octobot with 
    oscilattory venting.
     
"""

from mecode import G
import MultiMaterial

g = G()

def init_G(strFile):
    print "Initializing G..."
    global g
    g = G(
        print_lines=False,
        outfile=strFile,
        # header = r"\\vfiler1.seas.harvard.edu\group0\jlewis\User Files\Fitzgerald\SoftRobots\MeCodeFiles\header.txt",
        # footer = r"\\vfiler1.seas.harvard.edu\group0\jlewis\User Files\Fitzgerald\SoftRobots\MeCodeFiles\footer.txt"
    )
    g.rename_axis(z=MultiMaterial.cur_tool)
    MultiMaterial.change_tool(MultiMaterial.cur_tool_index)
    return g