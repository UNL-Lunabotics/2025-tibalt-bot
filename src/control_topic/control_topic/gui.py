# TODO: update using code from last year
import rclpy
from std_msgs.msg import Int16, Int16MultiArray
import os
try:
    import tkinter as tk
except ImportError: 
    import tkinter as tk

class SliderPublisher:

    def __init__(self, master):
        self.master = master
        master.title("Tibalt Controls")  # Window title
        master.geometry("800x300")  # Set the window size
        master.configure(background='#d00000')  # Nebraska Cornhusker red for the main window background

        # Styling variables
        label_width = 25
        slider_length = 300
        padding = {'padx': 5, 'pady': 5}

        excavation_frame = tk.Frame(master, bg='#f0f0f0')
        excavation_frame.grid(row=0, column=0, padx=10, pady=10, sticky="nsew")

        tk.Label(excavation_frame, text="Excavation Controls", bg='#f0f0f0').grid(row=0, column=0, sticky="ew")
        
        tk.Label(excavation_frame, text="Excav. Actuator Speed", bg='#f0f0f0').grid(row=0, column=0, sticky="ew")
        self.excavation_actuator = tk.Scale(excavation_frame, length=slider_length, from_=0, to=127, 
                                            orient=tk.HORIZONTAL, command=self.update_excavation_actuator)

        self.excavation_actuator.set(127)
        self.excavation_actuator_value = 127
        self.excavation_actuator.grid(row=1, column=0, sticky="ew", **padding)

        tk.Label(excavation_frame, text="Excav. Spinner Speed", bg='#f0f0f0').grid(row=2, column=0, sticky="ew")
        self.excavation_spinner = tk.Scale(excavation_frame, length=slider_length, from_=0, to=127, 
                                            orient=tk.HORIZONTAL, command=self.update_excavation_spinner)

        self.excavation_spinner.set(127)
        self.excavation_spinner_value = 127
        self.excavation_spinner.grid(row=3, column=0, sticky="ew", **padding)

        def update_excavation_actuator(self, value):
            self.trencher_left_value = value
            self.pub_data()

        def update_excavation_spinner(self, value):
            self.trencher_right_value = value
            self.pub_data()