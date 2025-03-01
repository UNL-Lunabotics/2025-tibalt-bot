#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16, Int16MultiArray
import os
try:
    import tkinter as tk
except ImportError:  # Python 2.x present
    import Tkinter as tk

array = []

def sensorCallback(arr):
    global ir_sensor_value
    ir_sensor_value.set(str(arr.data[0]))
    global pressure_sensor_value
    pressure_sensor_value.set(str(arr.data[1]))

class SliderPublisher:

    def __init__(self, master):
        self.master = master
        master.title("Tobias Controls")  # Window title
        master.geometry("800x300")  # Set the window size
        master.configure(background='#d00000')  # Nebraska Cornhusker red for the main window background

        # Styling variables
        label_width = 25
        slider_length = 300
        padding = {'padx': 5, 'pady': 5}

        # Frame for Trencher Controls
        trencher_frame = tk.Frame(master, bg='#f0f0f0')  # Neutral background for the frame
        trencher_frame.grid(row=0, column=0, padx=10, pady=10, sticky="nsew")

        tk.Label(trencher_frame, text="Trencher Controls", bg='#f0f0f0').grid(row=0, column=0, sticky="ew")

        tk.Label(trencher_frame, text="Trencher Left", bg='#f0f0f0').grid(row=0, column=0, sticky="ew")
        self.trencher_left = tk.Scale(trencher_frame, length=slider_length, from_=0, to=63, orient=tk.HORIZONTAL,
                                      command=self.update_trencher_left)
        self.trencher_left.set(63)
        self.trencher_left_value = 63
        self.trencher_left.grid(row=1, column=0, sticky="ew", **padding)

        tk.Label(trencher_frame, text="Trencher Right", bg='#f0f0f0').grid(row=2, column=0, sticky="ew")
        self.trencher_right = tk.Scale(trencher_frame, length=slider_length, from_=0, to=63, orient=tk.HORIZONTAL,
                                       command=self.update_trencher_right)
        self.trencher_right.set(63)
        self.trencher_right_value = 63

        self.trencher_right.grid(row=3, column=0, sticky="ew", **padding)

        tk.Label(trencher_frame, text="Trencher Actuator Speed", bg='#f0f0f0').grid(row=4, column=0, sticky="ew")
        self.trencher_act = tk.Scale(trencher_frame, length=slider_length, from_=0, to=63, orient=tk.HORIZONTAL,
                                     command=self.update_trencher_act)
        self.trencher_act.set(63)
        self.trencher_act_value = 63
        self.trencher_act.grid(row=5, column=0, sticky="ew", **padding)

        # Frame for Autonomous Controls
        auton_frame = tk.Frame(master, bg='#f0f0f0')  # Neutral background for the frame
        auton_frame.grid(row=0, column=1, padx=10, pady=10, sticky="nsew")

        tk.Label(auton_frame, text="Autonomous Controls", bg='#f0f0f0').grid(row=0, column=0, sticky="ew")

        tk.Label(auton_frame, text="Exc Auton Drive", bg='#f0f0f0').grid(row=0, column=0, sticky="ew")
        self.exc_auton_drive = tk.Scale(auton_frame, length=slider_length, from_=0, to=63, orient=tk.HORIZONTAL,
                                        command=self.update_exc_auton_drive)
        self.exc_auton_drive.set(15)
        self.exc_auton_drive_value = 15
        self.exc_auton_drive.grid(row=1, column=0, sticky="ew", **padding)

        tk.Label(auton_frame, text="Exc Auton Shake", bg='#f0f0f0').grid(row=2, column=0, sticky="ew")
        self.exc_auton_shake = tk.Scale(auton_frame, length=slider_length, from_=0, to=63, orient=tk.HORIZONTAL,
                                        command=self.update_exc_auton_shake)
        self.exc_auton_shake.set(15)
        self.exc_auton_shake_value = 15
        self.exc_auton_shake.grid(row=3, column=0, sticky="ew", **padding)

        tk.Label(auton_frame, text="Auton Drive Time (s)", bg='#f0f0f0').grid(row=4, column=0, sticky="ew")
        self.exc_auton_drive_time = tk.Scale(auton_frame, length=slider_length, from_=0, to=60, orient=tk.HORIZONTAL,
                                             command=self.update_exc_auton_drive_time)
        self.exc_auton_drive_time.set(15)
        self.exc_auton_drive_time_value = 15
        self.exc_auton_drive_time.grid(row=5, column=0, sticky="ew", **padding)

        # Frame for Motor Encoders
        encoder_frame = tk.Frame(master, bg='#f0f0f0')  # Neutral background for the frame
        encoder_frame.grid(row=0, column=2, padx=10, pady=10, sticky="nsew")

        tk.Label(encoder_frame, text="Sensors/Encoders", bg='#f0f0f0').grid(row=0, column=0, sticky="ew")

        global ir_sensor_value
        global pressure_sensor_value

        tk.Label(encoder_frame, text="IR Sensor", bg='#f0f0f0').grid(row=1, column=0, sticky="ew")
        ir_sensor = tk.Label(encoder_frame, textvariable=ir_sensor_value, bg='#f0f0f0')
        ir_sensor.grid(row=2, column=0, sticky="ew")

        tk.Label(encoder_frame, text="Pressure Sensor", bg='#f0f0f0').grid(row=3, column=0, sticky="ew")
        pressure_sensor = tk.Label(encoder_frame, textvariable=pressure_sensor_value, bg='#f0f0f0')
        pressure_sensor.grid(row=4, column=0, sticky="ew")

        tk.Label(encoder_frame, text="Camera Selector", bg='#f0f0f0').grid(row=5, column=0, sticky="ew")

        self.camera_var = tk.IntVar(root, 1)
        self.camera_hopper = tk.Radiobutton(encoder_frame, text='Hopper', variable=self.camera_var, value=2, command=self.update_camera).grid(row=6, column=0, sticky="ew")
        self.camera_trencher = tk.Radiobutton(encoder_frame, text='Trencher', variable=self.camera_var, value=1, command=self.update_camera).grid(row=7, column=0, sticky="ew")

        # Configure the grid expansion
        master.grid_columnconfigure(0, weight=1)
        master.grid_columnconfigure(1, weight=1)
        master.grid_rowconfigure(0, weight=1)

        # ROS initialization
        self.pub_sliders = rospy.Publisher('slider_values', Int16MultiArray, queue_size=10)

        self.pub_data()

    def update_trencher_left(self, value):
        self.trencher_left_value = value
        self.pub_data()

    def update_trencher_right(self, value):
        self.trencher_right_value = value
        self.pub_data()

    def update_trencher_act(self, value):
        self.trencher_act_value = value
        self.pub_data()

    def update_exc_auton_drive(self, value):
        self.exc_auton_drive_value = value
        self.pub_data()

    def update_exc_auton_shake(self, value):
        self.exc_auton_shake_value = value
        self.pub_data()

    def update_exc_auton_drive_time(self, value):
        self.exc_auton_drive_time_value = value
        self.pub_data()

    def update_camera(self):
        if(self.camera_var.get() == 1): # trencher
            os.system("rosrun topic_tools mux_select mux_cam cam_1/color/image_raw/compressed")
        elif(self.camera_var.get() == 2): # hopper
            os.system("rosrun topic_tools mux_select mux_cam cam_2/color/image_raw/compressed")
        self.pub_data()

    def pub_data(self):
        array = [int(self.trencher_left_value), int(self.trencher_right_value), int(self.exc_auton_drive_value), int(self.exc_auton_shake_value), int(self.exc_auton_drive_time_value), int(self.trencher_act_value) , self.camera_var.get()]
        self.pub_sliders.publish(Int16MultiArray(data=array))        

    # def update_encoders(self, data):
        #TODO pull encoder values from ROS

if __name__ == "__main__":
    rospy.init_node("GUI")
    rate = rospy.Rate(10)
    rospy.Subscriber("sensor_pipe", Int16MultiArray, sensorCallback)

    root = tk.Tk()
    global ir_sensor_value
    global pressure_sensor_value

    ir_sensor_value = tk.StringVar()
    ir_sensor_value.set("-1")

    pressure_sensor_value = tk.StringVar()
    pressure_sensor_value.set("-1")

    os.system("source ~/Documents/fbispycam/catkin_ws/devel/setup.bash")
    os.system("rosrun topic_tools mux bothcam/color/image_raw/compressed cam_1/color/image_raw/compressed cam_2/color/image_raw/compressed mux:=mux_cam &")

    root.geometry("500x500")
    my_gui = SliderPublisher(root)
    root.mainloop()
    while not rospy.is_shutdown():
        rate.sleep()
    rospy.spin()