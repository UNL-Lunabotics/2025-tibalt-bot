import tkinter as tk
import cv2
from PIL import Image, ImageTk
import os
import glob

# This function looks for video camera devices on the system (usually under /dev/video* on Linux).
# It ignores RealSense metadata devices (which show up as extra entries) and returns only working camera paths.
def detect_v4l_cameras():
    v4l_devices = glob.glob("/dev/video*")  # Get list of all video devices
    cameras = []
    for device in sorted(v4l_devices):
        # Ignore RealSense metadata devices by checking the actual device path
        if "realsense" in os.path.realpath(device).lower():
            continue
        cap = cv2.VideoCapture(device)  # Try opening the camera
        if cap.isOpened():
            ret, _ = cap.read()  # Try reading a frame
            if ret:
                cameras.append(device)  # Add to list if successful
            cap.release()
    return cameras

# Opens a camera from a specific device path and sets standard settings (like resolution and frame rate).
def open_camera(device_path):
    cap = cv2.VideoCapture(device_path)  # Try opening the camera
    if not cap.isOpened():
        print(f"Failed to open {device_path}")
        return None

    # Set resolution to 640x360 (16:9 aspect ratio) for consistency and bandwidth efficiency
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)

    # Try setting frame rate to 15 FPS to reduce USB bandwidth usage 
    cap.set(cv2.CAP_PROP_FPS, 15)

    return cap

# Updates a label next to the slider and stores the value in a shared list so it can be used elsewhere in the program.
def update_slider_label(slider_var, value_label, index, motor_values):
    value_label.config(text=f"{slider_var.get()}%")  # Update the display text
    motor_values[index] = slider_var.get()  # Store value in the shared list

# This function creates a custom slider with a label and a manual-entry box so the user can control motor values easily.
def create_actuator_slider(parent, label_text, index, motor_values, app):
    frame = tk.Frame(parent, padx=10, pady=10)  # Outer frame for each slider block

    name_label = tk.Label(frame, text=label_text, font=("Consolas", 13))  # Label for the slider
    name_label.pack(side="top")

    slider_var = tk.IntVar()  # Variable that tracks slider value

    # Sub-frame to hold the slider and entry box
    slider_and_entry_frame = tk.Frame(frame)
    slider_and_entry_frame.pack(side="top", pady=5)

    # Create the horizontal slider
    slider_widget = tk.Scale(
        slider_and_entry_frame,
        from_=0,
        to=100,
        orient="horizontal",
        length=400,
        sliderlength=40,
        width=20,
        variable=slider_var
    )
    slider_widget.pack(side="top")

    # Frame for the manual entry (user types value manually)
    entry_frame = tk.Frame(slider_and_entry_frame)
    entry_frame.pack(side="top", pady=(5, 0))

    entry = tk.Entry(entry_frame, width=5, justify='center', font=("Consolas", 13))  # Entry box
    entry.pack(side="left")

    percent_label = tk.Label(entry_frame, text="%", font=("Consolas", 13))  # Static % label
    percent_label.pack(side="left", padx=(2, 0))

    # Called when user presses ENTER in the entry box
    def on_entry_return(event=None):
        try:
            value = int(entry.get())  # Get and validate input
            value = max(0, min(100, value))  # Clamp to 0-100
            slider_var.set(value)  # Sync slider and input values
        except ValueError:
            entry.delete(0, tk.END)
            entry.insert(0, str(slider_var.get()))
        app.focus_set()  # Remove focus from entry box

    entry.bind("<Return>", on_entry_return)  # Bind ENTER key to handler

    # Called whenever slider is moved to sync the entry box and update shared motor values
    def on_slider_change(*args):
        entry.delete(0, tk.END)
        entry.insert(0, str(slider_var.get()))
        motor_values[index] = slider_var.get()

    slider_var.trace_add("write", on_slider_change)  # Attach listener

    entry.insert(0, str(slider_var.get()))  # Set initial value

    return slider_var, frame

def main(args=None):
    # Create the main application window
    app = tk.Tk()
    app.title('Tibalt Control Panel')
    app.minsize(1200, 800)  # Minimum window size

    # When clicking outside input boxes, reset focus
    def clear_focus(event):
        widget = event.widget
        if not isinstance(widget, tk.Entry):
            app.focus_set()

    app.bind_all("<Button-1>", clear_focus)

    # Initial motor values for 3 sliders
    motor_values = [0, 0, 0]  # [Hopper, Excavator, Spin]

    # Initialize camera objects (dummy placeholders)
    front_camera = cv2.VideoCapture()
    back_camera = cv2.VideoCapture()
    hopper_camera = cv2.VideoCapture()
    cameras = [front_camera, back_camera, hopper_camera]  # Track all cameras in a list

    # Try to find all usable cameras
    available_cams = detect_v4l_cameras()
    if len(available_cams) < 3:
        print(f"Warning: Only {len(available_cams)} cameras detected!")
        available_cams = ['/dev/video0', '/dev/video1', '/dev/video2']  # Fallback

    # Assign actual camera streams to slots
    camera_labels = ["Front Camera", "Back Camera", "Hopper Camera"]
    for i, dev in enumerate(available_cams[:3]):
        cap = open_camera(dev)
        if cap and cap.isOpened():
            cameras[i].release()
            cameras[i] = cap
            if i == 0: front_camera = cap
            elif i == 1: back_camera = cap
            elif i == 2: hopper_camera = cap
            print(f"Opened {camera_labels[i]} at {dev}")
        else:
            print(f"Failed to open {camera_labels[i]} at {dev}")

    # Flag to control whether streaming is active
    cameras_active = False

    # Layout setup: split screen into top and bottom
    main_frame = tk.Frame(app)
    main_frame.pack(fill="both", expand=True)

    top_frame = tk.Frame(main_frame)
    top_frame.pack(fill="both", expand=True)

    # Front camera panel
    front_cam_frame = tk.Frame(top_frame, padx=5, pady=5, bg="white", relief=tk.SUNKEN, borderwidth=2)
    tk.Label(front_cam_frame, text="Front Camera", font=("Consolas", 15), bg="white").pack()
    front_camera_label = tk.Label(front_cam_frame, bg="black")  # Where the image is shown
    front_camera_label.pack()
    front_cam_frame.pack(side="left", fill="both", expand=True, padx=10, pady=10)

    # Back camera panel
    back_cam_frame = tk.Frame(top_frame, padx=5, pady=5, bg="white", relief=tk.SUNKEN, borderwidth=2)
    tk.Label(back_cam_frame, text="Back Camera", font=("Consolas", 15), bg="white").pack()
    back_camera_label = tk.Label(back_cam_frame, bg="black")
    back_camera_label.pack()
    back_cam_frame.pack(side="left", fill="both", expand=True, padx=10, pady=10)

    # Bottom half: Hopper cam + sliders
    bottom_frame = tk.Frame(main_frame)
    bottom_frame.pack(fill="both", expand=True)

    hopper_cam_frame = tk.Frame(bottom_frame, padx=5, pady=5, bg="white", relief=tk.SUNKEN, borderwidth=2)
    tk.Label(hopper_cam_frame, text="Hopper Camera", font=("Consolas", 15), bg="white").pack()
    hopper_camera_label = tk.Label(hopper_cam_frame, bg="black")
    hopper_camera_label.pack()
    hopper_cam_frame.pack(side="left", fill="both", expand=True, padx=10, pady=10)

    # Sliders section for actuators
    sliders_frame = tk.Frame(bottom_frame)
    sliders_frame.pack(side="left", fill="both", expand=True, padx=10, pady=10)

    # Create 3 actuator sliders and pack them
    hopper_slider_var, hopper_slider_widget = create_actuator_slider(sliders_frame, "Hopper Actuator", 0, motor_values, app)
    hopper_slider_widget.pack(fill="x", pady=5)

    excavation_slider_var, excavation_slider_widget = create_actuator_slider(sliders_frame, "Excavation Actuator", 1, motor_values, app)
    excavation_slider_widget.pack(fill="x", pady=5)

    spin_slider_var, spin_slider_widget = create_actuator_slider(sliders_frame, "Excavation Spin", 2, motor_values, app)
    spin_slider_widget.pack(fill="x", pady=5)

    # This function updates the camera preview panels with the latest video frames
    def update_camera_feeds():
        if not cameras_active:
            return

        for cam, label, size in [
            (front_camera, front_camera_label, (640, 360)),
            (back_camera, back_camera_label, (640, 360)),
            (hopper_camera, hopper_camera_label, (512, 288))
        ]:
            if cam and cam.isOpened():
                ret, frame = cam.read()
                if ret:
                    img = cv2.resize(frame, size)
                    img = ImageTk.PhotoImage(image=Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB)))
                    label.config(image=img)
                    label.img = img  # Prevent garbage collection
                else:
                    label.config(text="Camera Disconnected")
            else:
                label.config(text="Camera Not Available")

        app.after(100, update_camera_feeds)  # Repeat after 100 ms

    # Start camera streaming and reconnect if necessary
    def start_camera_streams():
        nonlocal cameras_active, front_camera, back_camera, hopper_camera
        if not cameras_active:
            for i, cam in enumerate([front_camera, back_camera, hopper_camera]):
                if cam and not cam.isOpened() and i < len(available_cams):
                    new_cam = open_camera(available_cams[i])
                    if new_cam:
                        if i == 0: front_camera = new_cam
                        elif i == 1: back_camera = new_cam
                        else: hopper_camera = new_cam
            cameras_active = True
            update_camera_feeds()

    # Stop all camera streams and clear image previews
    def stop_camera_streams():
        nonlocal cameras_active
        cameras_active = False
        for label in [front_camera_label, back_camera_label, hopper_camera_label]:
            label.config(image='', text='Camera Stream Stopped')

    # Proper cleanup when window is closed
    def handle_closing():
        stop_camera_streams()
        for cam in [front_camera, back_camera, hopper_camera]:
            if cam and cam.isOpened():
                cam.release()
        app.destroy()

    app.protocol("WM_DELETE_WINDOW", handle_closing)
    start_camera_streams()
    app.mainloop()

if __name__ == '__main__':
    main()
