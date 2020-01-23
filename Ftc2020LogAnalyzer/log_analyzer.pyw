import sys
import gui
import logging
import traceback
import time
import os
import tkinter as tk
from tkinter import messagebox
from util import ParseError, RobotPose, Log, inside, str_get_vars, parse_file

try:
    fp = open(sys.argv[1])
except IndexError:
    fp = None
except FileNotFoundError:
    fp = None
    messagebox.showerror("Error", "Could not find the specified log file.")

try:
    win = gui.AnalysisWindow((500, 500), (24*6, 24*6), (18, 18))
    if fp:
        win.reload(fp)
    win.main_loop()
except Exception as e:
    # error logging and stuff
    fname = "crash_log.txt"
    try:
        os.remove(fname)
    except FileNotFoundError:
        pass
    logging.basicConfig(filename="crash_log.txt", level=logging.DEBUG)
    logging.critical("Traceback:\n{}{}: {}".format("".join(traceback.format_tb(e.__traceback__)), e.__class__.__name__, str(e)))
    messagebox.showerror("Error", "Something unexpectedly went wrong. A crash log file was created.")
finally:
    if fp:
        fp.close()