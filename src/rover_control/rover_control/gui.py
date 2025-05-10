# Tibalt Robot Control Panel
# This application provides a graphical interface to:
# 1. Display live camera feeds from multiple sources
# 2. Control robot actuators via sliders
# 3. Provide visual feedback of system status

import rclpy
from rclpy.qos import QoSProfile
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

import tkinter as tk
import cv2
from PIL import Image, ImageTk
import numpy as np
import pyrealsense2 as rs

class AppConfig:
    # Configuration class storing all application settings and constants
    # Dictionary mapping camera IDs to their display names
    CAMERA_NAMES = {
        'front': "Front Cam",  # Front-facing camera
        'back': "Back Cam",    # Rear-facing camera
        'hopper': "Hopper Cam" # Camera monitoring the material hopper
    }
    
    # Placeholder text to display when cameras are unavailable
    PLACEHOLDER_TEXTS = {
        'front': "Front Camera Not Available",
        'back': "Back Camera Not Available",
        'hopper': "Hopper Camera Not Available"
    }
    
    # Display dimensions for each camera feed
    CAMERA_SIZES = {
        'front': (640, 360),  # Width x Height in pixels
        'back': (640, 360),
        'hopper': (512, 288)
    }
    
    # Names for the control sliders
    SLIDER_NAMES = [
        "Hopper Actuator",  # Controls the hopper mechanism
        "Excavation Actuator",  # Controls the digging arm
        "Excavation Spin"  # Controls the rotation of the digging mechanism
    ]

class CameraBase:
    # Base class for all camera types
    # Provides common interface and basic functionality
    
    def __init__(self):
        # Initialize camera with default disconnected state
        self.connected = False  # Tracks if camera is successfully connected
    
    def release(self):
        # Safely release camera resources
        # Should be called when closing the application
        if self.connected:
            # Release resources based on camera type
            if hasattr(self, 'cap'):  # For Webcam
                self.cap.release()
            if hasattr(self, 'pipeline'):  # For RealSense
                self.pipeline.stop()
        self.connected = False  # Mark as disconnected

class Webcam(CameraBase):
    # Class for standard USB webcam interface
    # Inherits from CameraBase for common functionality
    
    def __init__(self, camera_number):
        # Initialize webcam connection
        # Parameters:
        #   camera_number (int): Index of the camera device (typically 0 for first camera)
        super().__init__()
        self.cap = cv2.VideoCapture(camera_number)  # Create OpenCV video capture object
        self.connected = self.cap.isOpened()  # Check if camera opened successfully
        
        if self.connected:
            # Set camera resolution (may not work on all cameras)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    
    def read(self):
        # Capture a frame from the webcam
        # Returns:
        #   tuple: (success_flag, frame)
        #          success_flag (bool): True if frame was captured successfully
        #          frame (numpy array): Captured image in BGR format
        if not self.connected:
            return False, None  # Early return if camera isn't connected
        
        ret, frame = self.cap.read()  # Attempt to read a frame
        
        # Convert grayscale to color if needed
        if ret and len(frame.shape) == 2:
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
            
        return ret, frame

class RealSenseCamera(CameraBase):
    # Class for Intel RealSense depth cameras
    # Inherits from CameraBase for common functionality
    
    def __init__(self):
        # Initialize RealSense camera (connection attempted on first read)
        super().__init__()
        self.pipeline = None  # Will hold the RealSense pipeline
        self.config = None    # Will hold the camera configuration
    
    def _initialize(self):
        # Attempt to connect to the RealSense camera
        # Called automatically on first frame read if not already connected
        try:
            self.pipeline = rs.pipeline()  # Create pipeline object
            self.config = rs.config()      # Create configuration object
            
            # Get the display dimensions for front camera from config
            width, height = AppConfig.CAMERA_SIZES['front']
            
            # Configure color stream
            self.config.enable_stream(
                rs.stream.color,  # Stream type (color video)
                width, height,    # Resolution
                rs.format.bgr8,   # Color format (OpenCV compatible)
                30                # Frames per second
            )
            
            # Start the pipeline with our configuration
            self.pipeline.start(self.config)
            self.connected = True  # Mark as connected
            
        except Exception as e:
            print(f"RealSense connection failed: {e}")
            self.connected = False
    
    def read(self):
        # Capture a frame from the RealSense camera
        # Returns:
        #   tuple: (success_flag, frame)
        #          success_flag (bool): True if frame was captured successfully
        #          frame (numpy array): Captured image in BGR format
        if not self.connected:
            self._initialize()  # Try to connect if not already connected
            if not self.connected:
                return False, None  # Return failure if still not connected
        
        try:
            # Wait for the next set of frames from the camera
            frames = self.pipeline.wait_for_frames()
            
            # Get the color frame from the set
            color_frame = frames.get_color_frame()
            
            if not color_frame:
                return False, None  # Return failure if no color frame
            
            # Convert the frame to a numpy array that OpenCV can use
            return True, np.asanyarray(color_frame.get_data())
            
        except Exception as e:
            print(f"RealSense read error: {e}")
            return False, None

class ControlPanel:
    # Main application class that creates and manages the GUI
    # Handles camera displays, control sliders, and user interaction
    
    def __init__(self, root_window):
        # Initialize the control panel application
        # Parameters:
        #   root_window (tk.Tk): The main tkinter window object
        self.root = root_window
        self.root.title('Tibalt Control Panel')  # Set window title
        self.root.minsize(1200, 800)  # Set minimum window dimensions
        self.running = True  # Flag to control the application loop
        
        # Dictionary to manage all camera feeds
        self.cameras = {
            'front': {
                'camera': RealSenseCamera(),  # Front camera instance
                'label': None  # Will hold the tkinter label for display
            },
            'back': {
                'camera': RealSenseCamera(),  # Back camera instance
                'label': None
            },
            'hopper': {
                'camera': Webcam(0),  # Hopper camera (first webcam)
                'label': None
            }
        }
        
        # Create placeholder images for when cameras are unavailable
        self.placeholders = self._create_placeholder_images()
        
        # Initialize slider values (0-100)
        self.slider_values = [0, 0, 0]  # Current slider positions
        self.slider_data_array = [0, 0, 0]  # Values to be sent to ROS2
        
        # Set up the user interface components
        self._setup_user_interface()

    def _create_placeholder_images(self):
        # Generate placeholder images to display when cameras are unavailable
        # Returns:
        #   dict: Dictionary of PhotoImage objects for each camera
        placeholders = {}
        
        for cam_key in ['front', 'back', 'hopper']:
            # Get the display dimensions for this camera
            width, height = AppConfig.CAMERA_SIZES[cam_key]
            
            # Create a blank gray image (50 = dark gray)
            image = np.full((height, width, 3), 50, dtype=np.uint8)
            
            # Get the placeholder text for this camera
            text = AppConfig.PLACEHOLDER_TEXTS[cam_key]
            
            # Set up text properties
            font = cv2.FONT_HERSHEY_SIMPLEX  # Font type
            text_size = cv2.getTextSize(text, font, 0.8, 2)[0]  # Get text dimensions
            
            # Calculate text position (centered)
            text_x = (width - text_size[0]) // 2
            text_y = (height + text_size[1]) // 2
            
            # Add the text to the image
            cv2.putText(
                image,  # Target image
                text,  # Text to display
                (text_x, text_y),  # Position
                font,  # Font type
                0.8,  # Font scale
                (255, 255, 255),  # Color (white)
                2  # Thickness
            )
            
            # Convert to format tkinter can display
            placeholders[cam_key] = ImageTk.PhotoImage(Image.fromarray(image))
            
        return placeholders

    def _create_camera_display(self, parent_frame, title):
        # Create a frame for displaying a camera feed
        # Parameters:
        #   parent_frame (tk.Frame): The container for this display
        #   title (str): The title to display above the camera feed
        # Returns:
        #   tuple: (frame, label)
        #          frame: The created frame widget
        #          label: The image display label
        frame = tk.Frame(parent_frame, relief=tk.SUNKEN, borderwidth=2)
        tk.Label(frame, text=title).pack()
        image_label = tk.Label(frame)
        image_label.pack()
        return frame, image_label

    def _create_slider_control(self, parent_frame, name, slider_index):
        # Create a slider control with associated widgets
        # Parameters:
        #   parent_frame (tk.Frame): The container for this control
        #   name (str): The name of the control to display
        #   slider_index (int): Which position in the values array this controls
        # Returns:
        #   tk.Frame: The frame containing all the slider widgets
        frame = tk.Frame(parent_frame)
        tk.Label(frame, text=name, font=("Arial", 12)).pack(anchor='w')
        
        control_frame = tk.Frame(frame)
        slider_var = tk.IntVar()
        slider = tk.Scale(
            control_frame,
            from_=0,
            to=100,
            orient="horizontal",
            variable=slider_var,
            length=250
        )
        slider.pack(side='left', padx=5)
        
        entry_var = tk.StringVar()
        entry = tk.Entry(
            control_frame,
            width=5,
            textvariable=entry_var,
            validate="key",
            validatecommand=(parent_frame.register(lambda text: text.isdigit() or text == ""), '%P')
        )
        entry.pack(side='left', padx=5)
        
        def handle_enter(event):
            # Handle Enter key in the text entry
            # Updates the slider when user types a value
            try:
                value = int(entry_var.get())
                if 0 <= value <= 100:  # Validate range
                    slider_var.set(value)  # Update slider
            except ValueError:
                pass  # Ignore invalid entries
        
        entry.bind('<Return>', handle_enter)
        control_frame.pack()
        
        value_label = tk.Label(frame, text="0%", font=("Arial", 12))
        value_label.pack()
        
        def update_display(*args):
            # Callback for when slider value changes
            # Updates the display and stores the value
            value = slider_var.get()
            value_label.config(text=f"{value}%")
            self.slider_values[slider_index] = value
            self.slider_data_array[slider_index] = value
            # TODO: Add ROS2 publishing code here to publish self.slider_data_array
        
        slider_var.trace_add("write", update_display)
        return frame

    def _setup_user_interface(self):
        # Create and arrange all GUI components
        main_frame = tk.Frame(self.root)
        main_frame.pack(fill='both', expand=True)
        
        camera_frame = tk.Frame(main_frame)
        camera_frame.pack(fill='both', expand=True)
        
        for cam_key in ['front', 'back']:
            frame, label = self._create_camera_display(
                camera_frame,
                AppConfig.CAMERA_NAMES[cam_key]
            )
            frame.pack(side='left', expand=True, padx=10, pady=10)
            self.cameras[cam_key]['label'] = label
        
        hopper_slider_frame = tk.Frame(main_frame)
        hopper_slider_frame.pack(fill='both', expand=True)
        
        frame, label = self._create_camera_display(
            hopper_slider_frame,
            AppConfig.CAMERA_NAMES['hopper']
        )
        frame.pack(side='left', expand=True, padx=10, pady=10)
        self.cameras['hopper']['label'] = label
        
        sliders_frame = tk.Frame(hopper_slider_frame)
        sliders_frame.pack(side='left', expand=True, padx=10, pady=10)
        
        for i, name in enumerate(AppConfig.SLIDER_NAMES):
            slider_frame = self._create_slider_control(sliders_frame, name, i)
            slider_frame.pack(fill='x', pady=10)

    def update_camera_views(self):
        # Update all camera displays with the latest frames
        # Runs continuously while the application is open
        if not self.running:
            return
        
        for cam_key in ['front', 'back', 'hopper']:
            camera = self.cameras[cam_key]['camera']
            label = self.cameras[cam_key]['label']
            placeholder = self.placeholders[cam_key]
            
            if camera.connected:
                success, frame = camera.read()
                
                if success:
                    if cam_key == 'hopper':
                        frame = cv2.resize(frame, AppConfig.CAMERA_SIZES['hopper'])
                    
                    img = ImageTk.PhotoImage(Image.fromarray(frame))
                    label.config(image=img)
                    label.image = img
                    continue
            
            label.config(image=placeholder)
            label.image = placeholder
        
        self.root.after(10, self.update_camera_views)

    def on_close(self):
        # Cleanup procedure when closing the application
        self.running = False
        for cam_info in self.cameras.values():
            cam_info['camera'].release()
        self.root.destroy()

class GUI(Node):
    """The node class for all the main Tibalt logic. This contains the functionality
    for drive train and autonomy."""

def main():
    # Main entry point for the application
    root = tk.Tk()
    app = ControlPanel(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    app.update_camera_views()
    root.mainloop()

if __name__ == '__main__':
    main()