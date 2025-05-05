import tkinter as tk
import cv2
from PIL import Image, ImageTk
import numpy as np

# Attempt to import the RealSense library; if it is not installed, set a flag to indicate its absence
try:
    import pyrealsense2 as rs
    REALSENSE_AVAILABLE = True  # RealSense library is available
except ImportError:
    REALSENSE_AVAILABLE = False  # RealSense library is not available

# This function updates the numeric value label next to each slider when the slider's value changes
def update_label(slider_value_var, value_label):
    # Update the value label to show the slider's current value as a percentage
    value_label.config(text=f"{slider_value_var.get()}%")

# This function creates a labeled slider with a display for its current value
# It returns the IntVar that tracks the slider's value and the frame containing the slider
def create_slider(parent, name):
    # Create a frame to contain the slider and its label
    slider_frame = tk.Frame(parent, padx=10, pady=5)

    # Create and pack a label showing the name of the actuator (e.g., "Hopper Actuator")
    name_label = tk.Label(slider_frame, text=name, font=("Arial", 12))
    name_label.pack()

    # Create an IntVar to store the current value of the slider
    slider_value_var = tk.IntVar()

    # Create the slider widget (horizontal orientation, range from 0 to 100)
    slider = tk.Scale(
        slider_frame,
        from_=0,
        to=100,
        orient="horizontal",
        length=200,
        width=15,
        variable=slider_value_var
    )
    slider.pack()

    # Create and pack a label to show the current value of the slider (initially 0%)
    value_label = tk.Label(slider_frame, text="0%", font=("Arial", 12))
    value_label.pack()

    # Set up a trace to update the label with the slider's value when it changes
    slider_value_var.trace_add("write", lambda *args: update_label(slider_value_var, value_label))

    # Return the variable tracking the slider's value and the frame containing the slider
    return slider_value_var, slider_frame

# This function creates a placeholder image to be displayed when a camera feed is not available
# The placeholder is a solid gray image with a message
def create_placeholder_image(width, height, text):
    # Create a black image with specified width and height
    image = np.zeros((height, width, 3), dtype=np.uint8)
    image[:] = (50, 50, 50)  # Fill background with dark gray

    # Add the specified text in the center of the image
    font = cv2.FONT_HERSHEY_SIMPLEX
    text_size = cv2.getTextSize(text, font, 0.8, 2)[0]
    text_x = (width - text_size[0]) // 2  # Calculate the x-coordinate to center the text
    text_y = (height + text_size[1]) // 2  # Calculate the y-coordinate to center the text
    cv2.putText(image, text, (text_x, text_y), font, 0.8, (255, 255, 255), 2)

    # Convert the image from OpenCV format to a PhotoImage object
    return ImageTk.PhotoImage(image=Image.fromarray(image))

# This class manages the initialization and frame retrieval for a RealSense camera
# It handles situations where no camera is connected or an error occurs
class RealSenseCamera:
    def __init__(self):
        self.pipeline = None
        self.config = None
        self.connected = False

        if REALSENSE_AVAILABLE:
            try:
                # Attempt to initialize the RealSense camera
                ctx = rs.context()
                if len(ctx.devices) > 0:  # Check if there are any connected RealSense devices
                    self.pipeline = rs.pipeline()
                    self.config = rs.config()
                    self.config.enable_stream(rs.stream.color, 640, 360, rs.format.bgr8, 30)  # Set the resolution and frame rate
                    self.pipeline.start(self.config)  # Start the camera pipeline
                    self.connected = True  # Set connected flag to True if camera initialized successfully
            except Exception as e:
                print(f"RealSense initialization error: {e}")  # Print error message if RealSense fails to initialize

    # Captures and returns the latest color frame as a numpy array
    def read(self):
        if not self.connected:
            return False, None  # Return False if the camera is not connected

        try:
            # Retrieve frames from the pipeline and extract the color frame
            frames = self.pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                return False, None  # Return False if no color frame is available

            frame_array = np.asanyarray(color_frame.get_data())  # Convert the frame data to a numpy array
            return True, frame_array  # Return True and the frame data if successful
        except Exception as e:
            print(f"RealSense read error: {e}")  # Print error message if reading the frame fails
            return False, None

    # Stops the camera pipeline to release hardware resources
    def release(self):
        if self.connected and self.pipeline:
            self.pipeline.stop()  # Stop the camera pipeline
            self.connected = False  # Set connected flag to False

# Main function that sets up the entire GUI, initializes cameras, and starts the frame update loop
def main(args=None):
    app = tk.Tk()
    app.title('Bot Control')  # Set the window title
    app.minsize(1200, 800)  # Set the minimum window size

    # This function handles cleanup when the GUI window is closed
    def on_closing():
        stop_camera_update_loop()  # Stop the camera feed update loop
        if front_camera.connected:
            front_camera.release()  # Release the front camera if it was initialized
        if back_camera.connected and back_camera != front_camera:
            back_camera.release()  # Release the back camera if it is different from the front camera
        app.destroy()  # Close the application window

    app.protocol("WM_DELETE_WINDOW", on_closing)  # Set the function to be called when the window is closed

    # Initialize front and back camera instances
    front_camera = RealSenseCamera()
    back_camera = RealSenseCamera() if REALSENSE_AVAILABLE else front_camera

    # Create placeholder images to use if the cameras are not available
    placeholder_front_camera = create_placeholder_image(640, 360, "Front Camera Not Available")
    placeholder_back_camera = create_placeholder_image(640, 360, "Back Camera Not Available")
    placeholder_hopper_camera = create_placeholder_image(512, 288, "Hopper Camera Placeholder")

    # Flag to track whether camera feed update loop is running
    camera_update_running = False

    # Set up the main container that holds all interface components
    main_container = tk.Frame(app)
    main_container.pack(fill="both", expand=True)

    # Create a container for the camera display sections
    camera_display_container = tk.Frame(main_container)
    camera_display_container.pack(fill="both", expand=True)

    # Set up the front camera display section
    front_camera_frame = tk.Frame(camera_display_container, padx=5, pady=5, bg="white", relief=tk.SUNKEN, borderwidth=2)
    tk.Label(front_camera_frame, text="Front Cam", font=("Arial", 14), bg="white").pack()
    front_camera_display_label = tk.Label(front_camera_frame, bg="black")
    front_camera_display_label.pack()
    front_camera_frame.pack(side="left", fill="both", expand=True, padx=10, pady=10)

    # Set up the back camera display section
    back_camera_frame = tk.Frame(camera_display_container, padx=5, pady=5, bg="white", relief=tk.SUNKEN, borderwidth=2)
    tk.Label(back_camera_frame, text="Back Cam", font=("Arial", 14), bg="white").pack()
    back_camera_display_label = tk.Label(back_camera_frame, bg="black")
    back_camera_display_label.pack()
    back_camera_frame.pack(side="left", fill="both", expand=True, padx=10, pady=10)

    # Create a section for the hopper camera and actuator sliders
    hopper_and_sliders_container = tk.Frame(main_container)
    hopper_and_sliders_container.pack(fill="both", expand=True)

    # Set up the hopper camera display section
    hopper_camera_frame = tk.Frame(hopper_and_sliders_container, padx=5, pady=5, bg="white", relief=tk.SUNKEN, borderwidth=2)
    tk.Label(hopper_camera_frame, text="Hopper Cam", font=("Arial", 14), bg="white").pack()
    hopper_camera_display_label = tk.Label(hopper_camera_frame, bg="black", image=placeholder_hopper_camera)
    hopper_camera_display_label.image = placeholder_hopper_camera  # Store the image to avoid garbage collection
    hopper_camera_display_label.pack()
    hopper_camera_frame.pack(side="left", fill="both", expand=True, padx=10, pady=10)

    # Create sliders for controlling actuators (hopper, excavation, spin)
    sliders_container = tk.Frame(hopper_and_sliders_container)
    sliders_container.pack(side="left", fill="both", expand=True, padx=10, pady=10)

    # Create sliders for each actuator
    hopper_slider_value, hopper_slider_frame = create_slider(sliders_container, "Hopper Actuator")
    hopper_slider_frame.pack(fill="x", pady=5)

    excavator_slider_value, excavator_slider_frame = create_slider(sliders_container, "Excavation Actuator")
    excavator_slider_frame.pack(fill="x", pady=5)

    spin_slider_value, spin_slider_frame = create_slider(sliders_container, "Excavation Spin")
    spin_slider_frame.pack(fill="x", pady=5)

    # Function that continuously updates the displayed camera frames
    def update_camera_frames():
        if not camera_update_running:
            return  # Exit if the update loop is not running

        # Update the front camera feed
        if front_camera.connected:
            front_cam_success, front_frame_array = front_camera.read()  # Read the front camera frame
            if front_cam_success:
                # If successful, update the displayed image
                front_image_tk = ImageTk.PhotoImage(image=Image.fromarray(front_frame_array))
                front_camera_display_label.config(image=front_image_tk)
                front_camera_display_label.photo_image = front_image_tk
            else:
                # If not successful, show the placeholder image
                front_camera_display_label.config(image=placeholder_front_camera)
                front_camera_display_label.photo_image = placeholder_front_camera
        else:
            # If no front camera is connected, show the placeholder image
            front_camera_display_label.config(image=placeholder_front_camera)
            front_camera_display_label.photo_image = placeholder_front_camera

        # Update the back camera feed
        if back_camera.connected and back_camera != front_camera:
            back_cam_success, back_frame_array = back_camera.read()  # Read the back camera frame
            if back_cam_success:
                # If successful, update the displayed image
                back_image_tk = ImageTk.PhotoImage(image=Image.fromarray(back_frame_array))
                back_camera_display_label.config(image=back_image_tk)
                back_camera_display_label.photo_image = back_image_tk
            else:
                # If not successful, show the placeholder image
                back_camera_display_label.config(image=placeholder_back_camera)
                back_camera_display_label.photo_image = placeholder_back_camera
        else:
            # If no back camera is connected, show the placeholder image
            back_camera_display_label.config(image=placeholder_back_camera)
            back_camera_display_label.photo_image = placeholder_back_camera

        # Continue updating camera frames every 10ms
        app.after(10, update_camera_frames)

    # Starts the camera frame update loop
    def start_camera_update_loop():
        nonlocal camera_update_running
        if not camera_update_running:
            camera_update_running = True
            update_camera_frames()

    # Stops the camera frame update loop and clears images
    def stop_camera_update_loop():
        nonlocal camera_update_running
        camera_update_running = False
        front_camera_display_label.config(image='')
        back_camera_display_label.config(image='')

    # Start the frame update loop
    start_camera_update_loop()

    # Run the Tkinter event loop
    app.mainloop()

if __name__ == '__main__':
    main()  # Run the main function to start the application