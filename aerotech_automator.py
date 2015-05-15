import json

import numpy as np

from mecode import G
from mecode.devices.keyence_profilometer import KeyenceProfilometer
from mecode.devices.keyence_micrometer import KeyenceMicrometer


ROBOMAMA_AXES_DATA = {
    'A': {
        'number': 4,
        'alignment_location': (586.075, 367.82),
    },
    'B': {
        'number': 5,
        'alignment_location': (482.075, 367.82),
    },
    'C': {
        'number': 6,
        'alignment_location': (378.075, 367.82),
    },
    'D': {
        'number': 7,
        'alignment_location': (299.075, 367.82),
    },
}


class AerotechAutomator(object):
    
    def __init__(self, axes=None, substrates={}, profilometer_axis='D', feed=40,
                 middle_search_start=-85, heaven=-0.5,
                 sensor_groove_offset=(-184.917, -23.485), 
                 z_constant=42.880204, z_ref_location=(-20, -50),
                 profile_feed=15, calfile_path="calfile.cal", 
                 axes_data=ROBOMAMA_AXES_DATA):
        # List of axis names to be used.
        self.axes = axes if axes is not None else []
        # Dictionary containing information about the substrates
        self.substrates = substrates
        # Axis that profilometer is on ex. 'A'
        self.profilometer_axis = profilometer_axis
        # Feed rate for too many things
        self.feed = feed
        # Feed rate for translations during profiling
        self.profile_feed = profile_feed
        # Starting point for finding middle reading range for profilometer
        self.middle_search_start = middle_search_start
        # The place you go when you die. Also the safe height for xy translation
        self.heaven = heaven
        # xy distance between the micrometer corsshairs and the alignment groove
        # crosshairs
        self.sensor_groove_offset = sensor_groove_offset
        # z-distance between the bottom of the z-micrometer reading range, and
        # the granite at z-ref
        self.z_constant = z_constant
        # The xy location for the granite reference point relative to the
        # groove crosshair, with respect to the profilometer.
        self.z_ref_location = z_ref_location
        # The path to write the calfile to
        self.calfile_path = calfile_path
        # Dict containing alignment data and ID number for all axes.
        self.axes_data = axes_data
         
        # x and y locations in absolute coordinates where the profilometer found 
        # the alignment grooves. If the find_alignment_grooves() function has
        # not been run, then this will be None.
        self.groove_crosshair = None
        # The position of the profilometer axis that will cause its reading to
        # be 0 at the z_ref_location. This value is written by calls to
        # self.find_z_ref()
        self.z_ref = None
        # A dict mapping substrate name to the position of the profilometer axis
        # that will cause the profilometer to read 0 at the origin of that
        # substrate (lower left).
        self.substrate_refs = {}
        # Dict mapping axis name to xyz tuple that would position nozzle tip
        # directly on the granite at the z_ref_location. Populate this dict
        # with calls to zero_nozzle().
        self.home_positions = {}
        # Dict containing the positions that will put each of the nozzles over
        # each of the substrates.
        self.substrate_origins = {}
        # Dict mapping substrate name to a profile of its surface, along with
        # its origin (relative to the profilometer axis) and step size.
        self.substrate_profiles = {}
       
    def setup(self):
        self.g = G(print_lines=False, direct_write=True)
        self.kp = KeyenceProfilometer('COM3') #com3
        self.km = KeyenceMicrometer('COM8')
        self.g.write('POSOFFSET CLEAR X Y U A B C D')
        
        
    def find_profilometer_middle(self, step=0.5, z_start='auto'):
        """ Take a reading from the profilometer, then move the axis with
        the profilometer head down by the step size. Repeats until it gets a 
        numerical reading from the profilometer. Then move by down by the
        numerical reading to get to the center of the profilometer range.
        
        Parameters
        ----------
        step : float
            The step size in between readings of the profilometer
        z_start : float
            The height to start searching for the middle at. The axis will move
            to this height to start. If set to 'auto' the middle_search_start
            setting will be used.
            
        Returns
        -------
        profilometer_middle : float
            The axis position that corresponds with a 0 reading on the
            profilometer
        """
        g = self.g
        # If the profilometer is already getting a reading we can short-circuit
        # this method and move directly to the read position.
        value = self.kp.read()
        if value is not None:
            pos = g.get_axis_pos(self.profilometer_axis)
            profilometer_middle = pos + value
            g.abs_move(**{self.profilometer_axis: profilometer_middle})
            return profilometer_middle
        
        floor = -100  # do not go below this height!
        dwell = 0.2
        axis_name = self.profilometer_axis
        feed = self.feed
        if z_start == 'auto':
            z_start = self.middle_search_start
        
        g.feed(feed)
        g.abs_move(**{axis_name: z_start})
        g.dwell(dwell)
        value = self.kp.read()
        while value is None:
            g.move(**{axis_name: -step})
            g.dwell(dwell)
            value = self.kp.read()
            pos = g.get_axis_pos(axis=axis_name)
            if (pos - step) <= floor:
                raise RuntimeError('Profilometer about to exceed set floor')
        g.move(**{axis_name: value})
        g.dwell(1)
        value = self.kp.read()
        pos = g.get_axis_pos(axis_name)
        profilometer_middle = pos + value
        g.abs_move(**{axis_name: profilometer_middle})
        return profilometer_middle
            
    
    def detect_edge(self, direction='+x', step=1, edge_tolerence=0.3,
                    find_middle=True):
        """ Move in the given direction until the surface height changes by more
        than `edge_tolerence`.
        
        Returns
        -------
        edge_position : float
            The position of the edge in the relevent coordinate
            
        """
        dwell = 0.2
        kp = self.kp
        g = self.g
        old_value = kp.read()    
        if find_middle is True or old_value is None:
            self.find_profilometer_middle()
            old_value = kp.read()
                
        sign = 1 if direction[0] == '+' else -1
        axis = direction[1]
        
        g.feed(self.feed)
        jump = 0
        while jump < edge_tolerence:
            g.move(**{axis: sign * step})
            g.dwell(dwell)
            new_value = kp.read()
            if new_value is None:
                break
            jump = abs(new_value - old_value)
            old_value = new_value
        g.dwell(dwell)
        edge_position = g.get_axis_pos(axis) - (float(sign) * step / 2)
        return edge_position
        
    def precise_detect_edge(self, direction='+x', large_step=1, small_step=0.2,
                            edge_tolerence=0.3, find_middle=True):
        """ Detect an edge in the given direction twice, once with low precision
        and high speed, then again with high precision.
        
        Returns
        -------
        edge_position : float
            The position of the edge in the relevent coordinate.
        
        """
        dwell = 0.2
        backstep = 4
        g = self.g
        
        sign = 1 if direction[0] == '+' else -1
        axis = direction[1]
        self.detect_edge(direction, large_step, edge_tolerence, find_middle)
        g.move(**{axis: -backstep * sign * large_step})
        g.dwell(dwell)
        edge_position = self.detect_edge(direction, small_step, edge_tolerence,
                                         False)
        return edge_position
    
    def find_left_bottom_substrate(self, known_position = 'current'):
        g = self.g
        if known_position == 'current':
            known_position = g.get_axis_pos('X'), g.get_axis_pos('Y')   
        else:
            self.go_to_heaven(self.profilometer_axis)
            g.abs_move(*known_position)
        x_left = self.precise_detect_edge(direction = '-x', large_step = 1, 
                    small_step = 0.2, edge_tolerance = 0.2, find_middle = True)
        g.abs_move(*known_position)
        y_bottom = self.precise_detect_edge(direction = '-y', large_step = 1, 
                    small_step = 0.2, edge_tolerance = 0.2, find_middle = False)
        x_low = x_left
        x_high = x_left + 76
        y_low = y_bottom
        y_high = y_low + 50.4
        return (x_low, x_high), (y_low, y_high)
      
        
    def find_substrate_bounds(self, known_position='current', dimension='x',
                              find_middle=True):
        """ Returns the lower and upper bounds of the substrate in the given
        dimension
        
        Parameters
        ----------
        known_position : tuple of floats (len 2) or str
            A known x, y point that places the measurement head over the
            substrate. If set to the string 'current' then the measurement head
            will be assumed to be over the substrate.
        dimension: str ('x', 'y', or 'both')
            Which dimension to find the edges in
        find_middle : bool
            If True then the middle of the profilometer range will be detected.
            
        Returns
        -------
        bounds : tuple of floats (len 2)
            The lower and upper bounds of the substrate.
        
        """
        g = self.g
        if known_position == 'current':
            known_position = g.get_axis_pos('X'), g.get_axis_pos('Y')   
        else:
            self.go_to_heaven(self.profilometer_axis)
            g.abs_move(*known_position)
        if dimension == 'both':
            x_low, x_high = self.find_substrate_bounds(known_position, 'x')
            g.abs_move(*known_position)
            y_low, y_high = self.find_substrate_bounds('current', 'y')
            return (x_low, x_high), (y_low, y_high)
        g.feed(self.feed)     
        pos_edge = self.precise_detect_edge('+' + dimension)
        g.abs_move(*known_position)
        neg_edge = self.precise_detect_edge('-' + dimension, find_middle=False)
        return neg_edge, pos_edge
        
    def find_alignment_grooves(self, h_start=(39.5, 330.5), v_start=(73, 319)):
        """ Find the absolute position of the alignment grooves in the mounting
        bracket and store them on self as groove_crosshair. If this method is
        called again the internally stored values will be returned.
        
        Parameters
        ----------
        h_start : tuple of floats (len 2)
            The x,y position to start seaching for the horizontal groove
        v_start : tuple  of floats (len 2)
            The x,y position to start searching for the verticle groove
        
        Returns
        -------
        x_edge, y_edge : tuple of floats (len 2)
            The x and y position of the alignment groves
            
        """
        g = self.g
        if self.groove_crosshair is not None:
            return self.groove_crosshair
        g.feed(self.feed)
        self.go_to_heaven()
        g.abs_move(*h_start)
        y_edge = self.precise_detect_edge('-y', large_step=0.5, small_step=0.1)
        g.abs_move(*v_start)
        x_edge = self.precise_detect_edge('-x', large_step=0.5, small_step=0.1,
                                    find_middle=False)
        self.groove_crosshair = x_edge, y_edge
        return x_edge, y_edge
    
    def find_z_ref(self):
        """ Find the axis position that will cause the profilometer to give a 0
        reading at the z_ref_location.
        
        Returns
        -------
        middle : float
            The axis position when the profilometer reads 0 over the ref point.
            
        Notes
        -----
        This method stores its result on self.z_ref
            
        """
        g = self.g
        g.feed(40)
        x_edge, y_edge = self.find_alignment_grooves()
        self.go_to_heaven()
        g.abs_move(x=x_edge + self.z_ref_location[0],
                   y=y_edge + self.z_ref_location[1])
        profilometer_middle = self.find_profilometer_middle()
        self.z_ref = profilometer_middle 
        return profilometer_middle
        
    def zero_all_nozzles(self):
        """ Call zero_nozzle() for all axes in self.axes.
        """
        for axis in self.axes:
            self.zero_nozzle(axis)
        
    def zero_nozzle(self, axis):
        """Sends a nozzle through the xyz alignment rig. First sends the nozzle
        to the appropriate xy location (x_start, y_start). Then, steps the 
        nozzle downwards until it breaks the plane of the sensor. It then moves
        nozzle into an ideal position for sensing. The sensor reads the distance
        in mm that the nozzle is from the central axis of the sensor. It outputs
        values in both x and y. Then the digital out switches the relays from 
        reading the y-sensor to the z-sensor. The nozzle is then slowly swept
        across the z-sensor reading plane. The program outputs the lowest value
        that is read during the sweep, as well as the axis position when that
        reading was taken. Shazzam!
        
        Parameters
        ----------
        axis : str ('A' 'B' 'C' or 'D')
            The axis that the nozzle of interest is mounted on
            
        Returns
        -------
        position : tuple of floats (len 3)
            The absolute position that would place the tip of the nozzle on the
            granite at the z ref point.
            
        Notes
        -----
        This method also updates self.home_positions with the value that is
        returned
        """
        zStart = -15 #Start position to find the point where nozzle breaks plane
        floor = -49.75 # minimum position that the axis will never break
        if axis == self.profilometer_axis:
            floor = -43.9286
        speed_fast = 10 #fast travel speed (mm/s)
        speed_slow = 2  #high accuracy travel speed (mm/s)
        zStep1 = 0.5    #step size for fast travel (mm)
        zStep2 = 0.1     #step size for slow travel (mm)
        backstep = 3.5   # Backstep in between fast and slow travel (mm)
        downstep = 0.4   # Downstep right before final reading (mm)
        dwell = 0        # dwell between readings (s)
        sweep_range = 1.5  # length of sweep range for z-measure (mm)
        sweep_speed = 0.1  # speed of sweep for z-measure (mm/s)
        
        if axis not in self.axes:
            self.axes.append(axis)
        start = self.axes_data[axis]['alignment_location']
        km = self.km
        g = self.g
        g.set_valve(num=7, value=0) #set relay for xy
        g.feed(40)
        self.go_to_heaven()
        km.get_xy() 
        #Initialize communication with keyence micrometer
        g.abs_move(*start)
        g.abs_move(**{axis:zStart})
        g.feed(speed_fast)
        value = km.read(1)
        while value is None:
            g.move(**{axis:-zStep1})
            g.dwell(dwell)
            value = km.read()
            pos=g.get_axis_pos(axis=axis)
            if (pos-zStep1)<floor:
                raise RuntimeError('next step will break through the set floor')
        g.move(**{axis:backstep})   
        g.feed(speed_slow)  
        value = km.read(1)
        while value is None:
            g.move(**{axis:-zStep2})
            g.dwell(dwell)
            value = km.read()
            pos=g.get_axis_pos(axis=axis)
            if (pos-zStep1)<floor:
                value = None
                raise RuntimeError('next step will break through the set floor')
        g.move(**{axis:-downstep})
        g.dwell(0.75)   
        
        (x_offset, y_offset) = km.read('both')
        g.set_valve(num = 7, value = 1)
        km.set_program(4)
        g.move(x=-x_offset, y=-y_offset)
        z_axis_position = g.get_axis_pos(axis=axis)
        g.dwell(3)
        dist = ((sweep_range)/1.414213)
        g.feed(10)
        g.move(x=-(dist/2), y=(dist/2))
        g.feed(sweep_speed)
        km.start_z_min()
        g.move(x=dist, y=-dist)
        z_min = km.stop_z_min()
        while z_min < 0.1:
            g.feed(15)
            g.move(x=-dist, y=dist)
            g.feed((sweep_speed/2))
            km.start_z_min()
            g.move(x=dist, y=-dist)
            z_min = km.stop_z_min()
        g.feed(25)
        g.abs_move(**{axis:-0.2})
        g.set_valve(num = 7, value = 0)
        x_center = start[0] - x_offset
        y_center = start[1] - y_offset
        z_bottom = z_axis_position - z_min
        x_groove = x_center + self.sensor_groove_offset[0]
        y_groove = y_center + self.sensor_groove_offset[1]
        z_granite = z_bottom - self.z_constant
        self.home_positions[axis] = (x_groove, y_groove, z_granite)
        return x_groove, y_groove, z_granite
        
    def profile_all_substrates(self):
        """ Profile every substrate in self.substrates.
        """
        for name, data in self.substrates.iteritems():
            if data['profile'] is False:
                continue
            self.profile_substrate(start=data['origin'], size=data['size'], 
                                    spacing=data['profile-spacing'], dwell=0.5,
                                    name=name)
         
    def profile_substrate(self, start, spacing, size='auto', dwell=0.5,
                          name=None):
        """ Find the surface topography using a profilometer.
        
        Parameters
        ----------
        start : tuple of floats (len 2)
            Bottom left of the area to profile. If `size` is set to 'auto' then
            this just needs to be a location known to place the profilometer
            over the substrate.
        spacing : tuple of floats (len 2)
            Spacing in x and y to take profilometer readings at
        size : 'auto' or tuple of floats (len 2)
            The width and height of the surface to sense. If set to 'auto' then
            the edges will automatically be detected.
        dwell : float
            Time to dwell at each point before taking a reading. The longer this
            is the less vibration will affect the sensed value.
        name : str or None
            The name of the substrate.
            
        Returns
        -------
        surface : 2D array
            A 2D array representing the surface
            
        Notes
        -----
        This method saves the surface array on self.substrate_profiles
        
        """
        inset = 1  # how far to inset when profiling an auto detected surface
        x_start, y_start = start
        x_spacing, y_spacing = spacing
        g = self.g
        g.feed(self.feed)
        self.go_to_heaven()
        if size == 'auto':
            g.abs_move(x_start, y_start)
            old_x_start = x_start
            x_start, x_stop = self.find_substrate_bounds(dimension='x')
            g.abs_move(old_x_start, y_start)
            y_start, y_stop = self.find_substrate_bounds(dimension='y',
                                                         find_middle=False)
            
            # Inset so we aren't measuring directly on the substrate bounds
            x_start, x_stop = x_start + inset, x_stop - inset
            y_start, y_stop = y_start + inset, y_stop - inset
            
        else:  # x_start and y_start is the bottom left of substrate
            x_stop = x_start + size[0]
            y_stop = y_start + size[1]
        
        g.abs_move(x_start, y_start)
        self.find_substrate_ref(name=name, position=(x_start, y_start),
                                safe=False)   
        x_range = np.arange(x_start, x_stop, x_spacing)
        y_range = np.arange(y_start, y_stop, y_spacing)
        surface = np.zeros((len(x_range), len(y_range)))
        g.feed(self.profile_feed)
        for i, x in enumerate(x_range):
            for j, y in enumerate(y_range):
                g.abs_move(x, y)
                g.dwell(dwell)
                value = None
                count = 0
                while value is None:
                    value = self.kp.read()
                    count += 1
                    if count > 10:
                        g.move(x=0.025)
                        count = 0
                surface[i, j] = value
        data = {'surface': surface, 'start': (x_start, y_start)}
        self.substrate_profiles[name] = data
        return surface
    
    def find_substrate_ref(self, name, position='auto', dwell=1, safe=True):
        """ Find the position of the profilometer axis that will cause the
        profilometer to read 0 at the given substrate position.
        
        Parameters
        ----------
        name : str
            Name of the substrate
        position : 'auto' or tuple of floats (len 2)
            xy position to get the reading at. If set to 'auto' the internal
            start position for the given substrate is used.
        safe : bool
            If True, axes will be homed before profiling
            
        Notes
        -----
        This method saves its data on self.substrate_refs
        
        """
        g = self.g
        if position == 'auto':
            position = self.substrates[name]['origin']
        if safe is True:
            self.go_to_heaven()
        g.abs_move(*position)
        g.dwell(dwell)
        middle = self.find_profilometer_middle()
        self.substrate_refs[name] = middle
        return middle
                            
    def write_cal_file(self, path, surface, spacing, offsets, axis='A',
                       mode='w+', ref_zero=True):
        """ Output a calibratin file used by the Aerotech stages
        
        Parameters
        ----------
        path : str
            Path to write the cal file to
        surface : 2d numpy array
            Array containing the profile data
        spacing : tuple of floats (len 2)
            Step size between points in the surface array (x, y)
        offsets : tuple of floats (len 2)
            The x, y offset to apply to the table. This is used to ensure the
            calibration is applied when the desired nozzle is over the
            calibration location.
        axis : str
            The axis this cal data should apply to.
        mode : str
            Mode to open the file in. 'w+' to create or overwrite existing file,
            'a' to append to file that already exists.
        ref_zero : bool
            If True, the surface will be offset to make position (0, 0) = 0
        
        """
        x_step, y_step = spacing
        x_offset, y_offset = offsets
        axis_num = self.axes_data[axis]['number']
        if ref_zero is True:
            surface -= surface[0, 0]
        surface = surface.T
        surface = -surface
        with open(path, mode) as f:
            num_cols = surface.shape[1]
            
            f.write(';        RowAxis  ColumnAxis  OutputAxis1  OutputAxis2  SampDistRow  SampDistCol  NumCols\n')  #noqa
            f.write(':START2D    2          1           1            2           {}          -{}          {}\n'.format(y_step, x_step, num_cols))  #noqa
            f.write(':START2D OUTAXIS3={} POSUNIT=PRIMARY CORUNIT=PRIMARY OFFSETROW = {} OFFSETCOL={}\n'.format(axis_num, y_offset, x_offset))  #noqa
            f.write(':START2D \n')
            for row in surface:
                for item in row:
                    f.write('0 0 ' + str(item) + '\t')
                f.write('\n')
                
            f.write(':END\n')
    
    def write_multi_table_cal_file(self, surface, spacing, substrate_name,
                                   filename=None, mode=None):
        if filename is None:
            filename = self.calfile_path
        
        origins = self.calculate_substrate_origins()
        for i, axis_name in enumerate(self.axes):
            if mode is None:
                if i == 0:
                    mode = 'w+'
                else:
                    mode = 'a'
            x_offset, y_offset, _ = origins[substrate_name][axis_name]
            self.write_cal_file(filename, surface, offsets=(x_offset, -y_offset),
                                axis=axis_name, mode=mode, spacing=spacing)
                                
    def write_master_cal_file(self):
        """ Write all the surface data into a multi-table cal file.
        """
        open(self.calfile_path, 'w+').close()  #clear the current calfile
        for name, data in self.substrates.iteritems():
            if data['profile'] is False:
                continue
            surface = self.substrate_profiles[name]['surface']
            self.write_multi_table_cal_file(surface=surface,
                                            spacing=data['profile-spacing'],
                                            substrate_name=name, mode='a')
        
    def calculate_substrate_origins(self):
        """ Calculate the positions that will put each of the nozzles over
        each of the substrates.
        
        Returns
        -------
        origins : dict
            Dict mapping substrate name to another dict of axis name to origin
            
        Notes
        -----
        This method also stores the returned dictionary on self.substrate_origins
        
        """
        if self.home_positions == {}:
            raise RuntimeError('No home positions found, call zero_nozzles() first')
        if self.groove_crosshair is None:
            raise RuntimeError('Alignment grooves not found, call find_alignment_grooves() first') 
        if self.substrate_refs.keys() != self.substrates.keys():
            raise RuntimeError('No substrate references found, call find_substrate_refs() first') 
        if self.z_ref is None:
            raise RuntimeError('z_ref not found, call find_z_ref() first')
        groove = self.groove_crosshair
        origins = {}
        for substrate_name in self.substrates:
            # If we have already profiled then used the start of the profile
            # array, otherwise use the user-defined substrate origin
            if substrate_name in self.substrate_profiles:
                start = self.substrate_profiles[substrate_name]['start']
            else:
                start = self.substrates[substrate_name]['origin']
            substrate_ref = self.substrate_refs[substrate_name]
            origins[substrate_name] = {}
            for axis_name in self.axes:
                home = self.home_positions[axis_name]
                x = home[0] + start[0] - groove[0]
                y = home[1] + start[1] - groove[1]
                z = home[2] + substrate_ref - self.z_ref
                origins[substrate_name][axis_name] = x, y, z
        self.substrate_origins = origins
        return origins
            
    def save_state(self, path):
        """ Save all the sensed and computed state to disk.
        
        Parameters
        ----------
        path : str
            Path of a file to write the data to
        
        """
        sanitized_profiles = {
            k: {'surface': v['surface'].tolist(), 'start': v['start']}
                for k, v in self.substrate_profiles.iteritems()}
        data = {
            'groove_crosshair': self.groove_crosshair,
            'z_ref': self.z_ref,
            'substrate_refs': self.substrate_refs,
            'home_positions': self.home_positions,
            'substrate_origins': self.substrate_origins,
            'substrate_profiles': sanitized_profiles,
            
            'axes': self.axes,
            'substrates': self.substrates,
            'profilometer_axis': self.profilometer_axis,
            'feed': self.feed,
            'profile_feed': self.profile_feed,
            'middle_search_start': self.middle_search_start,
            'heaven': self.heaven,
            'sensor_groove_offset': self.sensor_groove_offset,
            'z_constant': self.z_constant,
            'z_ref_location': self.z_ref_location,
            'calfile_path': self.calfile_path,
            'axes_data': self.axes_data,
            
        }
        with open(path, 'w') as f:
            print "Saving tip alignment to file " + str(path)
            json.dump(data, f, indent=4)
        
    def load_state(self, path):
        """ Load state from a file and store it on self.
        
        Parameters
        ----------
        path : str
            Path to a saved state file on disk.
        
        """
        with open(path) as f:
            d = json.load(f)
        if d['groove_crosshair']:
            self.groove_crosshair = d['groove_crosshair']
        if d['z_ref']:
            self.z_ref = d['z_ref']
        self.substrate_refs.update(d['substrate_refs'])
        self.home_positions.update(d['home_positions'])
        self.home_positions.update(d['home_positions'])
        self.substrate_origins.update(d['substrate_origins'])
        arrayified = {
            k: {'surface': np.array(v['surface']), 'start': v['start']}
                for k, v in d['substrate_profiles'].iteritems()}
        self.substrate_profiles.update(arrayified)
        
        self.axes = d['axes']
        self.substrates = d['substrates']
        self.profilometer_axis = d['profilometer_axis']
        self.feed = d['feed']
        self.profile_feed = d['profile_feed']
        self.middle_search_start = d['middle_search_start']
        self.heaven = d['heaven']
        self.sensor_groove_offset = d['sensor_groove_offset']
        self.z_constant = d['z_constant']
        self.z_ref_location = d['z_ref_location']
        self.calfile_path = d['calfile_path']
        self.axes_data = d['axes_data']
            
    def teardown(self):
        """ Close connections to the serial devices.
        """
        self.kp.disconnect()
        self.km.disconnect()
        self.g.teardown()
        
    def go_to_heaven(self):
        """ Move all nozzles to a safe height.
        """
        self.g.abs_move(A=self.heaven, B=self.heaven,
                        C=self.heaven, D=self.heaven)

    def view_substrate(self, substrate_name):
        """ Shows the surface plot of the given substrate
        
        Parameters
        ----------
        substrate_name : str
            The name of the substrate to show.
        """
        from mpl_toolkits.mplot3d import Axes3D  # noqa
        import matplotlib.pyplot as plt
        import numpy as np
        ax = plt.figure().gca(projection='3d')
        s = self.substrate_profiles[substrate_name]['surface']        
        x, y = np.meshgrid(np.arange(s.shape[0]), np.arange(s.shape[1]))
        ax.scatter(x.flat, y.flat, s.T.flat)
        plt.show()
                        
    def automate(self):
        """ Zero every nozzle and profile every substrate.
        """
        self.go_to_heaven()
        if self.substrates != {}:
            self.profile_all_substrates()
        self.find_alignment_grooves()    
        self.find_z_ref()
        self.zero_all_nozzles()
        if self.substrates != {}:
            self.write_master_cal_file()
            self.g.set_cal_file(self.calfile_path)
        self.go_to_heaven()
    
    def rezero_nozzles(self, nozzles, alignment_path = None, cal_file = False):
        """
        nozzles: list of strings ex ['A'] or ['A', 'B']
        alignment_path: string with path where alignment data is saved. Only put 
                        this in if you want rezeroing to automatically load and save
                                state.
        cal_file: Boolean - If true, the cal file will be rewritten to reflect the
                            offsets found for the new nozzles.
        """
        if alignment_path is not None:
            self.load_state(path = alignment_path)
        for nozzle in nozzles:
            self.zero_nozzle(nozzle)
        self.calculate_substrate_origins()
        if alignment_path is not None:
            self.save_state(path = alignment_path)   
        if cal_file is not False:
            self.write_master_cal_file()
            self.g.set_cal_file(self.calfile_path)
                
            