'''
Tibalt's GUI code      
Please dont breath on this code wrong or the whole thing will break
Tkinter is the worst :(
    
TODO: Fix aspect ratio issues with resolution scaling
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

# Try to import RealSense library, if available
try:
    import pyrealsense2 as rs
    REALSENSE_AVAILABLE = True  # Set flag if RealSense is available
except ImportError:
    REALSENSE_AVAILABLE = False  # Set flag if RealSense is not available

# Define color scheme and font settings for UI elements
BG_COLOR = "#f2f2f2"
FRAME_COLOR = "#ffffff"
ACCENT_COLOR = "#3498db"
TEXT_COLOR = "#2c3e50"
SUCCESS_COLOR = "#2ecc71"
WARNING_COLOR = "#e74c3c"
HEADER_FONT = ("Roboto", 20, "bold")
TITLE_FONT = ("Roboto", 14, "bold")

class CameraManager:
    # Class to manage camera resources and feed handling
    
    def __init__(self):
        # Initialize the CameraManager object, including setting up RealSense if available
        self.realsense_pipelines = []  # Store RealSense pipeline objects
        self.hopper_cam_placeholder = self.create_placeholder("HOPPER CAM\nNO FEED AVAILABLE")  # Placeholder for hopper camera
        
        # Initialize RealSense camera pipelines if available
        if REALSENSE_AVAILABLE:
            self.setup_realsense()
        
        # Create placeholders for front and rear cameras
        self.front_cam_placeholder = self.create_placeholder("FRONT CAM\nNO REALSENSE DETECTED")
        self.rear_cam_placeholder = self.create_placeholder("REAR CAM\nNO REALSENSE DETECTED")
    
    def create_placeholder(self, text):
        # Create a placeholder image with custom text for a camera feed
        width, height = 640, 360  # Placeholder dimensions
        img = np.zeros((height, width, 3), dtype=np.uint8)  # Create a black placeholder image
        lines = text.split('\n')  # Split the text into lines
        
        # Add the text to the placeholder image
        for i, line in enumerate(lines):
            y_pos = height // 2 - 30 + (i * 40)  # Calculate Y position for each line
            cv2.putText(img, line, (50, y_pos), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)  # Add text to the image
        
        return Image.fromarray(img)  # Convert numpy array to PIL Image

    def setup_realsense(self):
        # Initialize the RealSense pipeline for connected devices
        ctx = rs.context()  # Get RealSense context
        devices = ctx.query_devices()  # Get all connected devices
        
        for device in devices:
            try:
                pipeline = rs.pipeline()  # Create a new pipeline for each device
                config = rs.config()  # Set up the configuration for the pipeline
                config.enable_device(device.get_info(rs.camera_info.serial_number))  # Enable the device with serial number
                config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)  # Enable color stream at 640x480 resolution
                pipeline.start(config)  # Start the pipeline with the configuration
                self.realsense_pipelines.append(pipeline)  # Add pipeline to the list
            except Exception as e:
                print(f"RealSense error: {str(e)}")  # Handle errors during RealSense setup
    
    def get_front_frame(self):
        # Retrieve the current frame from the front RealSense camera
        if len(self.realsense_pipelines) > 0:
            try:
                frames = self.realsense_pipelines[0].wait_for_frames()  # Get frames from the first RealSense camera
                color_frame = frames.get_color_frame()  # Get the color frame
                if color_frame:
                    return np.asanyarray(color_frame.get_data())  # Return frame as numpy array
            except Exception as e:
                print(f"RealSense error: {str(e)}")  # Handle errors during frame capture
        return None  # Return None if no frame is available
    
    def get_rear_frame(self):
        # Retrieve the current frame from the rear RealSense camera
        if len(self.realsense_pipelines) > 1:
            try:
                frames = self.realsense_pipelines[1].wait_for_frames()  # Get frames from the second RealSense camera
                color_frame = frames.get_color_frame()  # Get the color frame
                if color_frame:
                    return np.asanyarray(color_frame.get_data())  # Return frame as numpy array
            except Exception as e:
                print(f"RealSense error: {str(e)}")  # Handle errors during frame capture
        return None  # Return None if no frame is available
    
    def release_resources(self):
        # Release all camera resources and stop pipelines
        for pipeline in self.realsense_pipelines:
            try:
                pipeline.stop()  # Stop the RealSense pipeline
            except:
                pass  # Ignore errors during resource release

def create_motor_slider(parent, motor_name):
    # Create a vertical motor control slider with labels displaying the motor's name and value
    frame = tk.Frame(parent, bg=FRAME_COLOR)  # Create a frame to hold the slider and labels
    
    # Split motor name into two parts if it's more than one word and create the label text
    name_lines = motor_name.split(" ")
    label_text = "\n".join(name_lines)  # Join the words with line breaks
    
    tk.Label(frame, text=label_text, font=TITLE_FONT, bg=FRAME_COLOR, justify="center").pack()  # Add label
    
    motor_var = tk.IntVar()  # Create a variable to store the slider value
    motor_slider = ttk.Scale(frame, from_=0, to=100, orient="vertical", variable=motor_var)  # Create the slider
    motor_slider.pack(pady=5)  # Pack the slider
    
    value_label = tk.Label(frame, text="0%", font=("Roboto", 12), bg=FRAME_COLOR)  # Label to display current value
    value_label.pack()  # Pack the value label
    
    # Update value label when the slider changes
    motor_var.trace_add("write", lambda *_: value_label.config(text=f"{motor_var.get()}%"))
    return frame  # Return the created slider frame

def create_camera_frame(parent, cam_title):
    # Create a frame for displaying a camera feed with a title
    frame = tk.Frame(parent, bg=FRAME_COLOR, bd=1, relief=tk.RIDGE)  # Create a frame for the camera
    tk.Label(frame, text=cam_title, font=TITLE_FONT, bg=FRAME_COLOR).pack(pady=5)  # Add camera title
    
    display_label = tk.Label(frame, bg="#e0e5ec")  # Create a label to display the camera feed
    display_label.pack(expand=True, fill='both', padx=5, pady=5)  # Pack the display label
    return frame, display_label  # Return the frame and label for later use

def setup_ui():
    # Set up the main application UI and create all widgets
    app_window = tk.Tk()  # Create the main window
    app_window.title('Tibalt Control Panel')  # Set the window title
    app_window.configure(bg=BG_COLOR)  # Set background color for the window
    app_window.geometry("1200x800")  # Set the window size
    
    # Create header with the title "TIBALT CONTROL PANEL"
    header_frame = tk.Frame(app_window, bg=ACCENT_COLOR, height=80)  # Create header frame
    header_frame.pack(fill='x')  # Pack the header frame to expand horizontally
    tk.Label(header_frame, text="TIBALT CONTROL PANEL", font=HEADER_FONT,
             bg=ACCENT_COLOR, fg="white").pack(side='left', padx=20)  # Add title label to header
    
    # Main content frame to hold camera and motor control sections
    main_content_frame = tk.Frame(app_window, bg=BG_COLOR)
    main_content_frame.pack(expand=True, fill='both', padx=15, pady=15)
    
    # Top section for camera feeds (front and rear cameras)
    top_camera_frame = tk.Frame(main_content_frame, bg=BG_COLOR)
    top_camera_frame.pack(fill='both', expand=True)
    
    # Front camera frame and display label
    front_cam_frame, front_cam_display = create_camera_frame(top_camera_frame, "FRONT CAMERA")
    front_cam_frame.pack(side='left', expand=True, fill='both', padx=10, pady=10)
    
    # Rear camera frame and display label
    rear_cam_frame, rear_cam_display = create_camera_frame(top_camera_frame, "REAR CAMERA")
    rear_cam_frame.pack(side='left', expand=True, fill='both', padx=10, pady=10)
    
    # Bottom section for hopper camera and motor controls
    bottom_section_frame = tk.Frame(main_content_frame, bg=BG_COLOR)
    bottom_section_frame.pack(fill='both', expand=True)
    
    # Hopper camera frame and display label
    hopper_cam_frame, hopper_cam_display = create_camera_frame(bottom_section_frame, "HOPPER CAMERA")
    hopper_cam_frame.pack(side='left', fill='both', expand=True, padx=10, pady=10)
    
    # Motor control section with sliders for various actuators
    motor_controls_frame = tk.Frame(bottom_section_frame, bg=FRAME_COLOR, bd=1, relief=tk.RIDGE)
    motor_controls_frame.pack(side='left', fill='y', padx=10, pady=10)
    
    # Label for motor control section
    tk.Label(motor_controls_frame, text="MOTOR CONTROLS", font=TITLE_FONT).pack(pady=10)
    
    # Frame to hold the motor sliders
    sliders_frame = tk.Frame(motor_controls_frame, bg=FRAME_COLOR)
    sliders_frame.pack(expand=True, pady=10)
    
    # Create sliders for hopper, excavator, and excavator spin controls
    hopper_slider = create_motor_slider(sliders_frame, "HOPPER ACTUATOR")
    hopper_slider.pack(side='left', expand=True, padx=10)
    
    excavator_slider = create_motor_slider(sliders_frame, "EXCAVATOR ACTUATOR")
    excavator_slider.pack(side='left', expand=True, padx=10)
    
    spin_slider = create_motor_slider(sliders_frame, "EXCAVATOR SPIN")
    spin_slider.pack(side='left', expand=True, padx=10)
    
    return app_window, front_cam_display, rear_cam_display, hopper_cam_display  # Return UI components

def update_displays(camera_mgr, front_display, rear_display, hopper_display):
    # Update the camera feed displays with the latest frames
    # Update front camera feed
    front_frame = camera_mgr.get_front_frame()  # Get front camera frame
    if front_frame is not None:
        front_image = Image.fromarray(cv2.cvtColor(front_frame, cv2.COLOR_BGR2RGB))  # Convert to PIL Image
    else:
        front_image = camera_mgr.front_cam_placeholder  # Use placeholder if no frame available
    
    # Update rear camera feed
    rear_frame = camera_mgr.get_rear_frame()  # Get rear camera frame
    if rear_frame is not None:
        rear_image = Image.fromarray(cv2.cvtColor(rear_frame, cv2.COLOR_BGR2RGB))  # Convert to PIL Image
    else:
        rear_image = camera_mgr.rear_cam_placeholder  # Use placeholder if no frame available
    
    # Hopper camera always uses placeholder
    hopper_image = camera_mgr.hopper_cam_placeholder
    
    # Resize and display images on the UI
    display_width, display_height = front_display.winfo_width()-10, front_display.winfo_height()-50  # Get display size
    for display_label, image in [(front_display, front_image), 
                                 (rear_display, rear_image), 
                                 (hopper_display, hopper_image)]:
        resized_image = image.resize((max(1, display_width), max(1, display_height)))  # Resize image to fit
        photo_image = ImageTk.PhotoImage(image=resized_image)  # Convert to PhotoImage for tkinter
        display_label.config(image=photo_image)  # Update display label with new image
        display_label.image = photo_image  # Keep reference to avoid garbage collection
    
    # Continue updating displays every 30ms
    front_display.master.after(30, update_displays, camera_mgr, front_display, rear_display, hopper_display)

def main():
    # Initialize the application and start the main event loop
    camera_mgr = CameraManager()  # Create a CameraManager instance
    app_window, front_display, rear_display, hopper_display = setup_ui()  # Set up the UI
    
    # Set initial window sizes and start updating the camera displays
    app_window.update()
    update_displays(camera_mgr, front_display, rear_display, hopper_display)
    
    # Set cleanup function on exit
    app_window.protocol("WM_DELETE_WINDOW", lambda: [camera_mgr.release_resources(), app_window.destroy()])
    
    # Start the main application event loop
    app_window.mainloop()

if __name__ == "__main__":
    main()  # Run the main function when the script is executed
