#!/usr/bin/env python3

import rospy
import argparse, sys
from std_msgs.msg import Float64MultiArray
import numpy as np
import tkinter as tk
import matplotlib as mpl
import matplotlib.pyplot as plt
mpl.use('TkAgg')
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2TkAgg

class MyPlot(tk.Tk):
    def __init__(self):
        tk.Tk.__init__(self)
        self.title('Laban Settings')

        self.happy_var = tk.IntVar()
        self.happy_ = tk.Checkbutton(self, text="happy", height=2, width=20, variable=self.happy_var, command=self.happy_callback).grid(row=0, column=1, sticky="w")
        self.sleepy_var = tk.IntVar()
        self.sleepy_ = tk.Checkbutton(self, text="sleepy", height=2, width=20, variable=self.sleepy_var, command=self.sleepy_callback).grid(row=1, column=1, sticky="w")
        self.grumpy_var = tk.IntVar()
        self.grumpy_ = tk.Checkbutton(self, text="grumpy", height=2, width=20, variable=self.grumpy_var, command=self.grumpy_callback).grid(row=2, column=1, sticky="w")


        self.t_p = tk.Label(self, bg='white', fg='black', height=2, width=20, text='Time')
        self.t_p.grid(row=0,column=0)
        self.s_p = tk.Label(self, bg='white', fg='black', height=2, width=20, text='Space')
        self.s_p.grid(row=2,column=0)
        self.i_p = tk.Label(self, bg='white', fg='black', height=2, width=20, text='Interaction')
        self.i_p.grid(row=4,column=0)
        
        self.time = tk.Scale(self, label='Time', from_=-1, to=1, orient=tk.HORIZONTAL, length=200, showvalue=0,tickinterval=2, resolution=0.01, command=self.time_selection)
        self.time.grid(row=1, column=0)
        self.space = tk.Scale(self, label='Space', from_=-1, to=1, orient=tk.HORIZONTAL, length=200, showvalue=0,tickinterval=2, resolution=0.01, command=self.space_selection)
        self.space.grid(row=3, column=0)
        self.Interaction = tk.Scale(self, label='Interaction', from_=-1, to=1, orient=tk.HORIZONTAL, length=200, showvalue=0,tickinterval=2, resolution=0.01, command=self.interaction_selection)
        self.Interaction.grid(row=5, column=0)
        
        self.deploy_button = tk.Button(self, bg='green', text='Deploy', height=2, width=25, command=self.deploy)
        self.deploy_button.grid(row=6, column=0)
        self.quit_button = tk.Button(self, bg='red', text='Quit', height=2, width=25, command=quit)
        self.quit_button.grid(row=6, column=1)
        
        self.time_val = 0.0
        self.space_val = 0.0
        self.interaction_val = 0.0
        self.happy_val, self.sleepy_val, self.grumpy_val = 0.0, 0.0, 0.0
        
        self.TSI_pub = rospy.Publisher('/TSI_values', Float64MultiArray, queue_size=10)
        self.TSI_msg = Float64MultiArray()
        
        toolbar_frame = tk.Frame(self)
        toolbar_frame.grid(row=7, column=0, sticky="w")

    def deploy(self):
        self.TSI_pub.publish(self.TSI_msg)
        
    def time_selection(self, val): 
        self.TSI_msg.data = [float(val), float(self.space_val), float(self.interaction_val), 0.0, 0.0, 0.0]
        self.t_p.config(text='Time = ' + val)
        self.time_val = val

    def space_selection(self, val):
        self.s_p.config(text='Space = ' + val)
        self.TSI_msg.data = [float(self.time_val), float(val) , float(self.interaction_val), 0.0, 0.0, 0.0]
        self.space_val = val

    def interaction_selection(self, val):
        self.i_p.config(text='Interaction = ' + val)
        self.TSI_msg.data = [float(self.time_val), float(self.space_val), float(val), 0.0, 0.0, 0.0]
        self.interaction_val = val   

    def happy_callback(self):
        self.happy_val = float(self.happy_var.get())
        self.TSI_msg.data = [0.0, 0.0, 0.0, self.happy_val, self.sleepy_val, self.grumpy_val]
                
    def sleepy_callback(self):
        self.sleepy_val = float(self.sleepy_var.get())
        self.TSI_msg.data = [0.0, 0.0, 0.0, self.happy_val, self.sleepy_val, self.grumpy_val]
        
    def grumpy_callback(self):
        self.grumpy_val = float(self.grumpy_var.get())     
        self.TSI_msg.data = [0.0, 0.0, 0.0, self.happy_val, self.sleepy_val, self.grumpy_val]
    
    def terminal_input_callback(self, args):
        if not args.robot_state:
            self.TSI_msg.data = [float(args.Time), float(args.Space), float(args.Interaction), 0.0, 0.0, 0.0]
        elif args.robot_state:
            if args.robot_state=='happy':
                self.TSI_msg.data = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0]
            if args.robot_state=='sleepy':
                self.TSI_msg.data = [0.0, 0.0, 0.0, 0.0, 1.0, 0.0]
            if args.robot_state=='grumpy':
                self.TSI_msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        self.TSI_pub.publish(self.TSI_msg)
        
if __name__ == '__main__':
    rospy.init_node("pygui", anonymous=True)
    parser = argparse.ArgumentParser()
    parser.add_argument('-t', action ='store', dest="Time", help='value of time setting from -1.0 to 1.0')
    parser.add_argument('-s', action ='store', dest="Space", help='value of space setting -1.0 to 1.0')
    parser.add_argument('-i', action ='store', dest="Interaction", help='value of interaction setting -1.0 to 1.0')
    parser.add_argument('-state', action ='store', dest="robot_state", help='enter robot state')
    args = parser.parse_args()
    app = MyPlot()
    rospy.sleep(3)
    if len(sys.argv)>1:
        app.terminal_input_callback(args)
    app.mainloop()





