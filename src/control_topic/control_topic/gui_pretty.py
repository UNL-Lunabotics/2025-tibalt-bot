'''
Tibalt's GUI code      
Please dont breath on this code wrong or the whole thing will break
Tkinter is the worst :(
    
TODO: Fix aspect ratio issues with resolution scaling 
        (and the weird thing that happens when you exit fullscreen)
      Decide between this GUI and the simplified version.
      Set up Realsense cameras to feed into the GUI
      Find out what kind of camera Hopper bin uses
      Implement ROS2 functionality
      The simplified gui might need window scaling if it is going to be used
'''

import tkinter as tk
from tkinter import ttk
import cv2
from PIL import Image, ImageTk
import numpy as np

# Try to import the RealSense camera library.
try:
    import pyrealsense2 as rs
    REALSENSE_AVAILABLE = True  # RealSense is available if we can import the library.
except ImportError:
    REALSENSE_AVAILABLE = False  # RealSense is not available if import fails.

# Set up colors and fonts for the GUI.
BG_COLOR = "#f2f2f2"        # Light gray background color for the window
FRAME_COLOR = "#ffffff"     # White color for frames around camera feeds and other sections
ACCENT_COLOR = "#3498db"    # Blue color used for accent elements like header
TEXT_COLOR = "#2c3e50"      # Dark gray text color for readability
HEADER_FONT = ("Roboto", 20, "bold")  # Font style for the header
TITLE_FONT = ("Roboto", 14, "bold")   # Font style for titles in the UI

class CameraManager:
    def __init__(self):
        # List to hold RealSense pipelines (each one connects to a camera)
        self.realsense_pipelines = []
        
        # Placeholder image for the hopper camera in case RealSense is not available
        self.hopper_cam_placeholder = self.create_placeholder("HOPPER CAM\nNO FEED AVAILABLE")
        
        # Set up RealSense cameras if available, otherwise use placeholders
        if REALSENSE_AVAILABLE:
            self.setup_realsense()
        
        # Placeholders for front and rear cameras if no RealSense is detected
        self.front_cam_placeholder = self.create_placeholder("FRONT CAM\nNO REALSENSE DETECTED")
        self.rear_cam_placeholder = self.create_placeholder("REAR CAM\nNO REALSENSE DETECTED")
    
    def create_placeholder(self, text):
        # Create a blank black image with text to indicate no feed is available
        width, height = 640, 360
        img = np.zeros((height, width, 3), dtype=np.uint8)  # Black image
        lines = text.split('\n')  # Split the placeholder text into multiple lines
        for i, line in enumerate(lines):
            y_pos = height // 2 - 30 + (i * 40)  # Calculate vertical position for each line of text
            cv2.putText(img, line, (50, y_pos),  # Put the text on the image at the calculated position
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)  # White text
        return Image.fromarray(img)  # Convert the image to a PIL format for use in Tkinter

    def setup_realsense(self):
        # Set up and start streaming from all connected RealSense devices
        ctx = rs.context()  # Create a RealSense context to access devices
        devices = ctx.query_devices()  # Query the connected RealSense devices
        for device in devices:
            try:
                pipeline = rs.pipeline()  # Create a pipeline for the device (camera)
                config = rs.config()  # Configuration for the camera stream
                config.enable_device(device.get_info(rs.camera_info.serial_number))  # Enable device using serial number
                config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)  # Enable color stream at 640x480 resolution, 30 fps
                pipeline.start(config)  # Start the pipeline to capture frames
                self.realsense_pipelines.append(pipeline)  # Add the pipeline to the list
            except Exception as e:
                print(f"RealSense error: {str(e)}")  # Print error message if setup fails
    
    def get_front_frame(self):
        # Get the latest frame from the first RealSense camera (front camera)
        if len(self.realsense_pipelines) > 0:
            try:
                frames = self.realsense_pipelines[0].wait_for_frames()  # Wait for the next available frame
                color_frame = frames.get_color_frame()  # Get the color frame
                if color_frame:
                    return np.asanyarray(color_frame.get_data())  # Convert the frame to a NumPy array
            except Exception as e:
                print(f"RealSense error: {str(e)}")  # Print error message if frame retrieval fails
        return None  # Return None if no frame is available
    
    def get_rear_frame(self):
        # Get the latest frame from the second RealSense camera (rear camera)
        if len(self.realsense_pipelines) > 1:
            try:
                frames = self.realsense_pipelines[1].wait_for_frames()  # Wait for the next available frame
                color_frame = frames.get_color_frame()  # Get the color frame
                if color_frame:
                    return np.asanyarray(color_frame.get_data())  # Convert the frame to a NumPy array
            except Exception as e:
                print(f"RealSense error: {str(e)}")  # Print error message if frame retrieval fails
        return None  # Return None if no frame is available
    
    def release_resources(self):
        # Stop all camera pipelines to release hardware resources when done
        for pipeline in self.realsense_pipelines:
            try:
                pipeline.stop()  # Stop the pipeline for each camera
            except:
                pass  # Ignore errors if stopping the pipeline fails

def create_motor_slider(parent, motor_name):
    # Create a vertical slider for controlling a motor's position or speed
    frame = tk.Frame(parent, bg=FRAME_COLOR)  # Create a frame for the motor control
    name_lines = motor_name.split(" ")  # Split the motor name into multiple lines if needed
    label_text = "\n".join(name_lines)  # Join the motor name lines with newline characters
    tk.Label(frame, text=label_text, font=TITLE_FONT, bg=FRAME_COLOR, justify="center").pack()  # Add the label
    
    motor_var = tk.IntVar()  # Create an integer variable to hold the slider value
    motor_slider = ttk.Scale(frame, from_=0, to=100, orient="vertical", variable=motor_var)  # Create the slider
    motor_slider.pack(pady=5)  # Add the slider to the frame with padding
    
    value_label = tk.Label(frame, text="0%", font=("Roboto", 12), bg=FRAME_COLOR)  # Label to display the slider value
    value_label.pack()  # Add the value label to the frame
    
    # Update the value label when the slider value changes
    motor_var.trace_add("write", lambda *_: value_label.config(text=f"{motor_var.get()}%"))
    return frame  # Return the frame containing the slider and label

def create_camera_frame(parent, cam_title):
    # Create a frame for displaying a camera feed with a title and display area
    frame = tk.Frame(parent, bg=FRAME_COLOR, bd=1, relief=tk.RIDGE)  # Create a frame for the camera feed
    tk.Label(frame, text=cam_title, font=TITLE_FONT, bg=FRAME_COLOR).pack(pady=5)  # Add a title label
    display_label = tk.Label(frame, bg="#e0e5ec")  # Label to display the camera feed
    display_label.pack(expand=True, fill='both', padx=5, pady=5)  # Pack the display label into the frame
    return frame, display_label  # Return the frame and the display label

def setup_ui():
    # Set up the full UI layout and return key parts for later updates
    app_window = tk.Tk()  # Create the main window for the application
    app_window.title('Tibalt Control Panel')  # Set the window title
    app_window.configure(bg=BG_COLOR)  # Set the background color of the window
    app_window.geometry("1200x800")  # Set the initial size of the window
    
    # Header section
    header_frame = tk.Frame(app_window, bg=ACCENT_COLOR, height=80)  # Create a header frame
    header_frame.pack(fill='x')  # Pack the header frame horizontally
    tk.Label(header_frame, text="TIBALT CONTROL PANEL", font=HEADER_FONT,
             bg=ACCENT_COLOR, fg="white").pack(side='left', padx=20)  # Add the header label
    
    main_content_frame = tk.Frame(app_window, bg=BG_COLOR)  # Create a frame for the main content
    main_content_frame.pack(expand=True, fill='both', padx=15, pady=15)  # Pack the main content frame
    
    # Top section with front and rear cameras
    top_camera_frame = tk.Frame(main_content_frame, bg=BG_COLOR)  # Create a frame for the camera feeds
    top_camera_frame.pack(fill='both', expand=True)  # Pack the top camera frame
    
    front_cam_frame, front_cam_display = create_camera_frame(top_camera_frame, "FRONT CAMERA")  # Create a frame for the front camera feed
    front_cam_frame.pack(side='left', expand=True, fill='both', padx=10, pady=10)  # Pack the front camera frame
    
    rear_cam_frame, rear_cam_display = create_camera_frame(top_camera_frame, "REAR CAMERA")  # Create a frame for the rear camera feed
    rear_cam_frame.pack(side='left', expand=True, fill='both', padx=10, pady=10)  # Pack the rear camera frame
    
    # Bottom section with hopper camera and motor controls
    bottom_section_frame = tk.Frame(main_content_frame, bg=BG_COLOR)  # Create a frame for the bottom section
    bottom_section_frame.pack(fill='both', expand=True)  # Pack the bottom section frame
    
    hopper_cam_frame, hopper_cam_display = create_camera_frame(bottom_section_frame, "HOPPER CAMERA")  # Create a frame for the hopper camera feed
    hopper_cam_frame.pack(side='left', fill='both', expand=True, padx=10, pady=10)  # Pack the hopper camera frame
    
    motor_controls_frame = tk.Frame(bottom_section_frame, bg=FRAME_COLOR, bd=1, relief=tk.RIDGE)  # Create a frame for motor controls
    motor_controls_frame.pack(side='left', fill='y', padx=10, pady=10)  # Pack the motor controls frame
    
    tk.Label(motor_controls_frame, text="MOTOR CONTROLS", font=TITLE_FONT).pack(pady=10)  # Add a label for motor controls
    
    sliders_frame = tk.Frame(motor_controls_frame, bg=FRAME_COLOR)  # Create a frame for the motor sliders
    sliders_frame.pack(expand=True, pady=10)  # Pack the sliders frame
    
    # Create individual sliders for different actuators
    hopper_slider = create_motor_slider(sliders_frame, "HOPPER ACTUATOR")
    hopper_slider.pack(side='left', expand=True, padx=10)  # Pack the hopper slider
    excavator_slider = create_motor_slider(sliders_frame, "EXCAVATOR ACTUATOR")
    excavator_slider.pack(side='left', expand=True, padx=10)  # Pack the excavator slider
    spin_slider = create_motor_slider(sliders_frame, "EXCAVATOR SPIN")
    spin_slider.pack(side='left', expand=True, padx=10)  # Pack the excavator spin slider
    
    return app_window, front_cam_display, rear_cam_display, hopper_cam_display  # Return key UI components

def update_displays(camera_mgr, front_display, rear_display, hopper_display):
    # Update the camera displays with the latest frames or placeholders
    front_frame = camera_mgr.get_front_frame()  # Get the latest frame from the front camera
    if front_frame is not None:
        front_image = Image.fromarray(cv2.cvtColor(front_frame, cv2.COLOR_BGR2RGB))  # Convert the frame to an image
    else:
        front_image = camera_mgr.front_cam_placeholder  # Use the placeholder if no frame is available
    
    rear_frame = camera_mgr.get_rear_frame()  # Get the latest frame from the rear camera
    if rear_frame is not None:
        rear_image = Image.fromarray(cv2.cvtColor(rear_frame, cv2.COLOR_BGR2RGB))  # Convert the frame to an image
    else:
        rear_image = camera_mgr.rear_cam_placeholder  # Use the placeholder if no frame is available
    
    hopper_image = camera_mgr.hopper_cam_placeholder  # Always use the hopper placeholder for now
    
    display_width, display_height = front_display.winfo_width()-10, front_display.winfo_height()-50  # Get the display area size
    for display_label, image in [(front_display, front_image),
                                 (rear_display, rear_image),
                                 (hopper_display, hopper_image)]:
        resized_image = image.resize((max(1, display_width), max(1, display_height)))  # Resize the image to fit the display area
        photo_image = ImageTk.PhotoImage(image=resized_image)  # Convert the image to a Tkinter-compatible format
        display_label.config(image=photo_image)  # Update the display label with the new image
        display_label.image = photo_image  # Keep a reference to the image to avoid garbage collection
    
    # Schedule the next update in about 33 milliseconds (30 FPS)
    front_display.master.after(30, update_displays, camera_mgr, front_display, rear_display, hopper_display)

def main():
    # Set up the camera manager and the UI, then start the Tkinter main loop
    camera_mgr = CameraManager()  # Create a camera manager to handle camera operations
    app_window, front_display, rear_display, hopper_display = setup_ui()  # Set up the user interface
    app_window.update()  # Update the window to make sure layout calculations are done
    update_displays(camera_mgr, front_display, rear_display, hopper_display)  # Start the display update loop
    app_window.protocol("WM_DELETE_WINDOW", lambda: [camera_mgr.release_resources(), app_window.destroy()])  # Clean up resources when the window is closed
    app_window.mainloop()  # Start the Tkinter main loop to keep the app running

if __name__ == "__main__":
    main()  # Run the main function to start the application
