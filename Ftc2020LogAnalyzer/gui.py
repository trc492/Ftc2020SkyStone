import pygame
import time
import tkinter as tk
from tkinter import filedialog, messagebox
import os
import platform
import numpy as np
from util import Stopwatch, ParseError, apply_alliance, rotate_vector, parse_file

class InfoWindow:
    def __init__(self, analysis_window):
        self.analysis_window = analysis_window
        self.root = tk.Tk()
        self.root.title("Extra info")
        self.root.resizable(False, False)
        self.format_str = "x: target: %.1f, input: %.1f, error: %.1f\ny: target: %.1f, input: %.1f, error: %.1f\nheading: target: %.1f, input: %.1f, error: %.1f"
        self.label = tk.Label(self.root, text=self.get_info_text())
        self.label.pack(side=tk.TOP)
        self.open = False
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.withdraw()

    def update(self):
        self.label.configure(text=self.get_info_text())
        self.root.update()

    def get_info_text(self):
        if self.analysis_window.log_info:
            x_t, y_t, h_t = self.analysis_window.log_info[self.analysis_window.step].rel_target.v3
            x_i, y_i, h_i = self.analysis_window.log_info[self.analysis_window.step].rel_input.v3
            x_e, y_e, h_e = self.analysis_window.log_info[self.analysis_window.step].rel_error.v3
        else:
            x_t, y_t, h_t, x_i, y_i, h_i, x_e, y_e, h_e = [0 for i in range(9)]
        return self.format_str % (x_t, x_i, x_e, y_t, y_i, y_e, h_t, h_i, h_e)
    
    def on_close(self):
        self.open = False
        self.root.withdraw()

    def reopen(self):
        if not self.open:
            self.open = True
            self.root.deiconify()
        else:
            self.root.focus_force()

class AnalysisWindow:
    def __init__(self, screen_dimensions, field_dimensions, robot_dimensions, log_name=None, log_info=None, alliance=None):
        self.screen_dimensions = screen_dimensions
        self.field_dimensions = field_dimensions
        self.robot_dimensions = robot_dimensions
        self.log_name = log_name
        self.log_info = log_info
        self.alliance = alliance

        self.kill = False
        
        # Need to have this seemingly useless array because otherwise Python garbage collects the images for some reason
        self.images = []

        # Used to prevent setting the slider position from calling an update on the timer value
        self.set_update = False

        self.step = 0

        self.stopwatch = Stopwatch(max_time=None, start_paused=True)
        self.root = tk.Tk()
        self.root.title("Tracelog analysis")
        self.root.resizable(False, False)

        self.info_window = InfoWindow(self)

        menu_bar = tk.Menu(self.root)
        menu_bar.add_command(label="Open", command=self.prompt_file)
        menu_bar.add_command(label="Close", command=self.murder)

        embed = tk.Frame(self.root, width=500, height=500)
        embed.pack(side=tk.TOP)

        self.buttonwin = tk.Frame(self.root, width=100, height=500)
        self.buttonwin.pack(side=tk.TOP)

        sliderwin = tk.Frame(self.root, width=600, height=50)
        sliderwin.pack(side=tk.BOTTOM)

        os.environ["SDL_WINDOWID"] = str(embed.winfo_id())
        if platform.system == "Windows":
            os.environ["SDL_VIDEODRIVER"] = "windib"

        self.screen = pygame.display.set_mode(self.screen_dimensions)
        self.screen.fill(pygame.Color(0, 100, 255))

        pygame.font.init()
        self.timer_font = pygame.font.SysFont("Courier New", 30)
        self.info_font = pygame.font.SysFont("Courier New", 15)
        pygame.display.init()
        pygame.display.update()

        self.robot_surface = pygame.image.load("assets\\robot.png").convert_alpha()

        self.background = pygame.image.load("assets\\field.png")

        self.add_image_button("assets\\jb_button.png", self.stopwatch.reset)
        self.add_image_button("assets\\b_button.png", self.prev_step)
        self.add_image_button("assets\\f_button.png", self.next_step)
        self.add_image_button("assets\\jf_button.png", self.stopwatch.max)
        self.add_image_button("assets\\play_button.png", self.stopwatch.start)
        self.add_image_button("assets\\pause_button.png", self.stopwatch.pause)
        self.add_image_button("assets\\stop_button.png", self.stopwatch.stop)
        self.add_image_button("assets\\info_button.png", self.info_window.reopen)

        self.time_slider = tk.Scale(sliderwin, command=self.slider_update, orient=tk.HORIZONTAL, length=500,
                                    resolution=0.001, from_=0.0, to=(log_info[-1].time if log_info else 0.0), showvalue=False)
        self.time_slider.pack(side=tk.BOTTOM)

        self.root.configure(menu=menu_bar)
        self.root.update()
    
    def murder(self):
        self.kill = True

    def add_image_button(self, image_path, command):
        image = tk.PhotoImage(master=self.root, file=image_path)
        self.images.append(image)
        button = tk.Button(self.buttonwin, image=image, command=command)
        button.pack(side=tk.LEFT, padx=(2, 2))

    def prompt_file(self):
        f = filedialog.askopenfile(parent=self.root, mode='r', title='Choose a file', filetypes=(("log files","*.log"),("all files","*.*")))
        if f != None:
            self.reload(f)
            # parse_file handles file closing so we don't have to do that here

    def reload(self, log_file):
        try:
            self.log_name, self.log_info, self.alliance = parse_file(log_file)
        except ParseError as e:
            messagebox.showerror("Error", "Something went wrong parsing the file: %s. Make sure the file is a valid autonomous log." % e.message)
            return
        self.stopwatch.max_time = self.log_info[-1].time
        self.stopwatch.stop()
        self.step = 0
        self.root.title("Tracelog analysis: " + self.log_name)
        self.info_window.root.title(self.log_name)
        self.time_slider.configure(to=self.log_info[-1].time)

    def slider_update(self, num):
        if self.set_update:
            self.set_update = False
        else:
            self.stopwatch.set_time(float(num))
            self.update_step()

    def render_text(self, text, x, y, font, spacing=5, color=(255, 255, 255)):
        lines = text.splitlines()
        for i, l in enumerate(lines):
            self.screen.blit(font.render(l, False, color), (x, y + (font.size("|")[1] + spacing)*i))

    def inches_to_pixels(self, coords):
        return (int(coords[0] * (self.screen_dimensions[0] / self.field_dimensions[0])), \
            int(coords[1] * (self.screen_dimensions[1] / self.field_dimensions[1])))

    def update_time_slider(self):
        self.set_update = True
        self.time_slider.set(self.stopwatch.get_time())

    def update_step(self):
        for i in range(len(self.log_info)):
            if self.log_info[i].time > self.stopwatch.get_time():
                self.step = i - 1 if i != 0 else i
                return
        self.step = len(self.log_info) - 1
    
    def draw_robot(self):
        x, y, angle = apply_alliance(self.alliance, self.field_dimensions, self.log_info[self.step].actual_pos.v3)
        robot_rect = self.robot_surface.get_rect()
        robot_rect.center = self.inches_to_pixels((x, y))
        self.screen.blit(pygame.transform.rotate(self.robot_surface, angle), robot_rect)

    def draw_robot_error(self):
        robot_pos = self.inches_to_pixels(apply_alliance(self.alliance, self.field_dimensions, self.log_info[self.step].actual_pos.pos))
        error_pose = self.log_info[self.step].get_abs_error(self.alliance, self.field_dimensions)
        error_pos = self.inches_to_pixels(error_pose.pos)
        heading_error_vector = self.inches_to_pixels(rotate_vector((0, 5), error_pose.heading, True))
        abs_heading_error = (error_pos[0] + heading_error_vector[0], error_pos[1] + heading_error_vector[1])
        pygame.draw.line(self.screen, (255,0,0), robot_pos, error_pos, 5)
        pygame.draw.line(self.screen, (0, 255, 0), error_pos, abs_heading_error, 5)

    def draw_timer(self):
        text_y = self.screen_dimensions[1] - 70 if self.alliance == "BLUE_ALLIANCE" else 40
        self.render_text("Time: %.3f" % self.stopwatch.get_time(), 15, text_y, self.timer_font)

    def draw_robot_info(self):
        last_time = self.log_info[self.step].time
        x, y, angle = self.log_info[self.step].actual_pos.v3
        state = self.log_info[self.step].state_name
        text_x = self.screen_dimensions[0] - max(self.info_font.size("state: " + state)[0], 215)
        text_y = self.screen_dimensions[1] - 100 if self.alliance == "BLUE_ALLIANCE" else 10
        self.render_text("x: %.1f\ny: %.1f\nangle: %.1f\nlast time: %.3f\nstate: %s" % (x, y, angle, last_time, state), \
            text_x, text_y, self.info_font, spacing=3)

    def next_step(self):
        if self.step != len(self.log_info) - 1:
            self.step += 1
        self.stopwatch.set_time(self.log_info[self.step].time)
    
    def prev_step(self):
        if self.step == 0:
            self.stopwatch.reset()
            self.update_step()
        else:
            self.step -= 1
            self.stopwatch.set_time(self.log_info[self.step].time)

    def main_loop(self):
        while(1):
            if self.kill:
                exit()
            self.screen.blit(self.background, (0,0))
            if self.log_name != None and self.log_info != None and self.alliance != None:
                self.update_step()
                self.update_time_slider()
                self.draw_robot()
                if self.info_window.open:
                    self.draw_robot_error()
                self.draw_robot_info()
                if self.info_window.open:
                    self.info_window.update()
            self.draw_timer()
            try:
                self.root.update()
            except tk.TclError:
                break
            pygame.display.update()