import tkinter as tk
import cv2
from PIL import Image, ImageTk
import numpy as np

# Attempt to import the RealSense library; if it is not installed, set a flag to indicate its absence
try:
    import pyrealsense2 as rs
    REALSENSE_AVAILABLE = True
except ImportError:
    REALSENSE_AVAILABLE = False

# This function updates the numeric value label next to each slider when the slider's value changes
def update_label(slider_value_var, value_label):
    value_label.config(text=f"{slider_value_var.get()}%")

# This function creates a labeled slider with a display for its current value
# It returns the IntVar that tracks the slider's value and the frame containing the slider
def create_slider(parent, name):
    slider_frame = tk.Frame(parent, padx=10, pady=5)

    name_label = tk.Label(slider_frame, text=name, font=("Arial", 12))
    name_label.pack()

    slider_value_var = tk.IntVar()

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

    value_label = tk.Label(slider_frame, text="0%", font=("Arial", 12))
    value_label.pack()

    slider_value_var.trace_add("write", lambda *args: update_label(slider_value_var, value_label))

    return slider_value_var, slider_frame

# This function creates a placeholder image using OpenCV
# The placeholder is useful for displaying a message when a camera feed is not available
def create_placeholder_image(width, height, text):
    image = np.zeros((height, width, 3), dtype=np.uint8)
    image[:] = (50, 50, 50)  # Fill background with dark gray

    font = cv2.FONT_HERSHEY_SIMPLEX
    text_size = cv2.getTextSize(text, font, 0.8, 2)[0]
    text_x = (width - text_size[0]) // 2
    text_y = (height + text_size[1]) // 2
    cv2.putText(image, text, (text_x, text_y), font, 0.8, (255, 255, 255), 2)

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
                ctx = rs.context()
                if len(ctx.devices) > 0:
                    self.pipeline = rs.pipeline()
                    self.config = rs.config()
                    self.config.enable_stream(rs.stream.color, 640, 360, rs.format.bgr8, 30)
                    self.pipeline.start(self.config)
                    self.connected = True
            except Exception as e:
                print(f"RealSense initialization error: {e}")

    # Captures and returns the latest color frame as a numpy array
    def read(self):
        if not self.connected:
            return False, None

        try:
            frames = self.pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                return False, None

            frame_array = np.asanyarray(color_frame.get_data())
            return True, frame_array
        except Exception as e:
            print(f"RealSense read error: {e}")
            return False, None

    # Stops the camera pipeline to release hardware resources
    def release(self):
        if self.connected and self.pipeline:
            self.pipeline.stop()
            self.connected = False

# Main function that sets up the entire GUI, initializes cameras, and starts the frame update loop
def main(args=None):
    app = tk.Tk()
    app.title('Bot Control')
    app.minsize(1200, 800)

    # This function handles cleanup when the GUI window is closed
    def on_closing():
        stop_camera_update_loop()
        if front_camera.connected:
            front_camera.release()
        if back_camera.connected and back_camera != front_camera:
            back_camera.release()
        app.destroy()

    app.protocol("WM_DELETE_WINDOW", on_closing)

    # Initialize front and back camera instances
    front_camera = RealSenseCamera()
    back_camera = RealSenseCamera() if REALSENSE_AVAILABLE else front_camera

    # Create placeholder images to use if the cameras are not available
    placeholder_front_camera = create_placeholder_image(640, 360, "Front Camera Not Available")
    placeholder_back_camera = create_placeholder_image(640, 360, "Back Camera Not Available")
    placeholder_hopper_camera = create_placeholder_image(512, 288, "Hopper Camera Placeholder")

    camera_update_running = False

    # Set up the main container that holds all interface components
    main_container = tk.Frame(app)
    main_container.pack(fill="both", expand=True)

    # Create a container for the camera display sections
    camera_display_container = tk.Frame(main_container)
    camera_display_container.pack(fill="both", expand=True)

    # Set up the front camera display
    front_camera_frame = tk.Frame(camera_display_container, padx=5, pady=5, bg="white", relief=tk.SUNKEN, borderwidth=2)
    tk.Label(front_camera_frame, text="Front Cam", font=("Arial", 14), bg="white").pack()
    front_camera_display_label = tk.Label(front_camera_frame, bg="black")
    front_camera_display_label.pack()
    front_camera_frame.pack(side="left", fill="both", expand=True, padx=10, pady=10)

    # Set up the back camera display
    back_camera_frame = tk.Frame(camera_display_container, padx=5, pady=5, bg="white", relief=tk.SUNKEN, borderwidth=2)
    tk.Label(back_camera_frame, text="Back Cam", font=("Arial", 14), bg="white").pack()
    back_camera_display_label = tk.Label(back_camera_frame, bg="black")
    back_camera_display_label.pack()
    back_camera_frame.pack(side="left", fill="both", expand=True, padx=10, pady=10)

    # Create a section for the hopper camera and actuator sliders
    hopper_and_sliders_container = tk.Frame(main_container)
    hopper_and_sliders_container.pack(fill="both", expand=True)

    # Set up the hopper camera display
    hopper_camera_frame = tk.Frame(hopper_and_sliders_container, padx=5, pady=5, bg="white", relief=tk.SUNKEN, borderwidth=2)
    tk.Label(hopper_camera_frame, text="Hopper Cam", font=("Arial", 14), bg="white").pack()
    hopper_camera_display_label = tk.Label(hopper_camera_frame, bg="black", image=placeholder_hopper_camera)
    hopper_camera_display_label.image = placeholder_hopper_camera
    hopper_camera_display_label.pack()
    hopper_camera_frame.pack(side="left", fill="both", expand=True, padx=10, pady=10)

    # Create sliders for controlling actuators
    sliders_container = tk.Frame(hopper_and_sliders_container)
    sliders_container.pack(side="left", fill="both", expand=True, padx=10, pady=10)

    hopper_slider_value, hopper_slider_frame = create_slider(sliders_container, "Hopper Actuator")
    hopper_slider_frame.pack(fill="x", pady=5)

    excavator_slider_value, excavator_slider_frame = create_slider(sliders_container, "Excavation Actuator")
    excavator_slider_frame.pack(fill="x", pady=5)

    spin_slider_value, spin_slider_frame = create_slider(sliders_container, "Excavation Spin")
    spin_slider_frame.pack(fill="x", pady=5)

    # Function that continuously updates the displayed camera frames
    def update_camera_frames():
        if not camera_update_running:
            return

        # Update front camera
        if front_camera.connected:
            front_cam_success, front_frame_array = front_camera.read()
            if front_cam_success:
                front_image_tk = ImageTk.PhotoImage(image=Image.fromarray(front_frame_array))
                front_camera_display_label.config(image=front_image_tk)
                front_camera_display_label.photo_image = front_image_tk
            else:
                front_camera_display_label.config(image=placeholder_front_camera)
                front_camera_display_label.photo_image = placeholder_front_camera
        else:
            front_camera_display_label.config(image=placeholder_front_camera)
            front_camera_display_label.photo_image = placeholder_front_camera

        # Update back camera
        if back_camera.connected and back_camera != front_camera:
            back_cam_success, back_frame_array = back_camera.read()
            if back_cam_success:
                back_image_tk = ImageTk.PhotoImage(image=Image.fromarray(back_frame_array))
                back_camera_display_label.config(image=back_image_tk)
                back_camera_display_label.photo_image = back_image_tk
            else:
                back_camera_display_label.config(image=placeholder_back_camera)
                back_camera_display_label.photo_image = placeholder_back_camera
        else:
            back_camera_display_label.config(image=placeholder_back_camera)
            back_camera_display_label.photo_image = placeholder_back_camera

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

    app.mainloop()

if __name__ == '__main__':
    main()
