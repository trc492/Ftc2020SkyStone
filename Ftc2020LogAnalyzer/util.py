import time
import math
import os
import traceback
import numpy as np
from tkinter import messagebox

class ParseError(Exception):
    def __init__(self, message):
        self.message = message

class Stopwatch:
    def __init__(self, max_time=None, start_paused=False):
        self.start_time = time.time()
        self.last_time = 0.0
        self.paused = start_paused
        self.max_time = max_time
    
    def get_time(self):
        if self.paused:
            if self.max_time == None:
                return self.last_time
            else:
                return min(self.last_time, self.max_time)
        else:
            if self.max_time != None and time.time() - self.start_time > self.max_time:
                self.paused = True
                self.last_time = self.max_time
                return self.max_time
            else:
                return time.time() - self.start_time

    def set_time(self, t):
        self.last_time = t
        self.start_time = time.time() - self.last_time

    def reset(self):
        self.start_time = time.time()
        self.last_time = 0.0

    def max(self):
        if self.max_time != None:
            self.start_time = time.time() - self.max_time
            self.last_time = self.max_time

    def pause(self):
        if not self.paused:
            self.last_time = self.get_time()
            self.paused = True

    def start(self):
        if self.paused:
            self.start_time = time.time() - self.last_time
            self.paused = False

    def stop(self):
        self.pause()
        self.reset()

class RobotPose:
    def __init__(self, x, y, heading):
        self.x = float(x)
        self.y = float(y)
        self.heading = float(heading)
        self.pos = (self.x, self.y)
        self.v3 = (self.x, self.y, self.heading)

class Log:
    def __init__(self, time, actual_pos, rel_target, rel_input, rel_error, state_name):
        self.time = time
        self.actual_pos = actual_pos
        self.rel_target = rel_target
        self.rel_input = rel_input
        self.rel_error = rel_error
        self.state_name = state_name

    def get_abs_error(self, alliance, field_dimensions):
        # orients the error to extend the actual robot pos, the heading is based off of the robot heading target, not the actual heading
        abs_x, abs_y, abs_heading = apply_alliance(alliance, field_dimensions, self.actual_pos.v3)
        abs_heading_corrected = abs_heading - self.rel_error.heading
        # negative y because pygame uses a right hand y axis, robot uses left
        rotated = rotate_vector((self.rel_error.x, -self.rel_error.y), abs_heading_corrected, True) 
        x = abs_x + rotated[0]
        y = abs_y + rotated[1]
        return RobotPose(x, y, (abs_heading - self.rel_error.heading + 180) % 360)

def inside(string, opening, closing):
    inside = ""
    is_inside = False
    for character in string:
        if is_inside:
            if character == closing:
                return inside
            else:
                inside += character
        else:
            if character == opening:
                is_inside = True
    raise ParseError("nothing inside %s and %s: %s" % (opening, closing, string))

def find_var(string, var_name):
    before = ": ([{/|\\,"
    after = " =:("
    var_counter = 0
    for i in range(len(string)):
        # If the characters match and we are at the beginning of the and if the character before, if it exists and is not part of the variable name, is an acceptable before character
        if string[i] == var_name[var_counter] and (var_counter != 0 or (i == 0 or string[i - 1] in before)):
            var_counter += 1
            # If we are at the end of the variable name
            if var_counter == len(var_name):
                # If the character after, if it exists, is an acceptable after character
                if (i == len(string) - 1 or string[i + 1] in after):
                    return string[i+1:]
                else:
                    var_counter = 0
        else:
            var_counter = 0
    raise ParseError("specified variable %s not in string: %s" % (var_name, string))

def str_get_vars(string, *var_names):
    parsed_vars = []
    starters = " =:("
    enders = " ,;\n):"
    for var_name in var_names:
        after = find_var(string, var_name)
        var = ""
        recording = False
        for i in range(len(after)):
            if recording:
                if after[i] in enders:
                    break
                else:
                    var += after[i]
            else:
                if after[i] not in starters:
                    recording = True
                    var += after[i]
        parsed_vars.append(var)
    return parsed_vars

def apply_alliance(alliance, field_dimensions, v3_or_pos):
    try:
        x, y, angle = v3_or_pos
    except ValueError:
        x, y = v3_or_pos
        angle = None
    if angle != None:
        angle *= -1
    if alliance == "BLUE_ALLIANCE":
        x = -x
        if angle != None:
            angle += 180
    else:
        y = field_dimensions[1] - y
    return (x, y) if angle == None else (x, y, angle)

def rotate_vector(v, angle, degrees=False):
    if degrees:
        angle = math.radians(angle)
    rot_matrix = np.array([[math.cos(angle), -math.sin(angle)],
                           [math.sin(angle),  math.cos(angle)]])
    rotated = np.matmul(np.array(v), rot_matrix)
    return tuple(rotated)

def parse_file(fp):
    try:
        lines = fp.readlines()
    except UnicodeDecodeError:
        raise ParseError("the file must be in text format")

    pos_info = []

    for i in range(len(lines)):
        if "RobotPose" in lines[i]:
            o = 0
            while(1):
                try:
                    last_time = inside(lines[i-o], "[", "]")
                    break
                except ParseError:
                    o += 1
            o = 0
            while(1):
                try:
                    state = str_get_vars(lines[i-o], ">>>>>")[0]
                    break
                except ParseError:
                    o += 1
            x_target, x_input, x_error = str_get_vars(lines[i+1], "Target", "Input", "Error")
            y_target, y_input, y_error = str_get_vars(lines[i+2], "Target", "Input", "Error")
            h_target, h_input, h_error = str_get_vars(lines[i+3], "Target", "Input", "Error")
            target_pose = RobotPose(x_target, y_target, h_target)
            input_pose = RobotPose(x_input, y_input, h_input)
            error_pose = RobotPose(x_error, y_error, h_error)
            # VS Code apparently hates asterisks
            x, y, h = str_get_vars(lines[i], "x", "y", "angle")
            absolute_pose = RobotPose(x, y, h)
            pos_info.append(Log(float(last_time), absolute_pose, target_pose, input_pose, error_pose, state))

    try:
        alliance = lines[1].split(" ")[2][:-1]
    except IndexError:
        raise ParseError("alliance line not formatted correctly")
    name = inside(lines[1], "[", "]")

    if len(pos_info) == 0:
        raise ParseError("no position info")

    return (name, pos_info, alliance)