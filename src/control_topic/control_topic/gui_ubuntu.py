import tkinter as tk
import cv2
from PIL import Image, ImageTk
import os
import glob

def detect_v4l_cameras():
    # Detects available V4L2 cameras while ignoring RealSense-specific devices.
    # Returns a list of camera device paths (e.g., ['/dev/video0', '/dev/video2']).
    v4l_devices = glob.glob("/dev/video*")
    cameras = []
    for device in sorted(v4l_devices):
        # Skip RealSense metadata devices by checking symlink paths
        if "realsense" in os.path.realpath(device).lower():
            continue
        cap = cv2.VideoCapture(device)
        if cap.isOpened():
            ret, _ = cap.read()
            if ret:
                cameras.append(device)
            cap.release()
    return cameras

def open_camera(device_path):
    # Opens a camera device with consistent settings.
    # Sets resolution to 640x360 and tries to set FPS to 15 for stability.
    # Returns the VideoCapture object or None if failed.
    cap = cv2.VideoCapture(device_path)
    if not cap.isOpened():
        print(f"Failed to open {device_path}")
        return None
    
    # Set standard resolution (RealSense cameras may default to higher)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)
    
    # Try to reduce FPS to lower USB bandwidth usage
    cap.set(cv2.CAP_PROP_FPS, 15)
    
    return cap

# Updates the text label showing the current value of the slider
# Also stores the value in an array so it can be accessed elsewhere
def update_slider_label(slider_var, value_label, index, motor_values):
    value_label.config(text=f"{slider_var.get()}%")
    motor_values[index] = slider_var.get() 

# Creates a slider widget with a label and manual-entry field
def create_actuator_slider(parent, label_text, index, motor_values, app):
    frame = tk.Frame(parent, padx=10, pady=10)

    name_label = tk.Label(frame, text=label_text, font=("Consolas", 13))
    name_label.pack(side="top")

    slider_var = tk.IntVar()

    # Frame for holding the slider and manual-entry field
    slider_and_entry_frame = tk.Frame(frame)
    slider_and_entry_frame.pack(side="top", pady=5)

    # Creates a horizontal slider
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

    # Frame for holding the manual entry field and a Percentage symbol
    entry_frame = tk.Frame(slider_and_entry_frame)
    entry_frame.pack(side="top", pady=(5, 0))

    # Initializes the manual entry field
    entry = tk.Entry(entry_frame, width=5, justify='center', font=("Consolas", 13))
    entry.pack(side="left")

    # Static percentage sign label
    percent_label = tk.Label(entry_frame, text="%", font=("Consolas", 13))
    percent_label.pack(side="left", padx=(2, 0))

    # Updates manual entry field and slider value on ENTER key press
    def on_entry_return(event=None):
        try:
            value = int(entry.get())
            value = max(0, min(100, value))  # Clamp value between 0-100
            slider_var.set(value)
        except ValueError:
            entry.delete(0, tk.END)
            entry.insert(0, str(slider_var.get()))
        # Remove focus from manual entry field on ENTER key press
        app.focus_set() 

    entry.bind("<Return>", on_entry_return)

    # Syncs slider movement with manual entry field
    def on_slider_change(*args):
        entry.delete(0, tk.END)
        entry.insert(0, str(slider_var.get()))
        motor_values[index] = slider_var.get()

    slider_var.trace_add("write", on_slider_change)

    # Initialize the entry with the default value (0)
    entry.insert(0, str(slider_var.get()))

    return slider_var, frame

def main(args=None):
    # Create main application window
    app = tk.Tk()
    app.title('Tibalt Control Panel')
    app.minsize(1200, 800)
    
    # Clears focus from Entry widgets when clicking anywhere else
    def clear_focus(event):
        widget = event.widget
        if not isinstance(widget, tk.Entry):
            app.focus_set()

    app.bind_all("<Button-1>", clear_focus)

    # Array to track actuator slider values
    motor_values = [0, 0, 0]  # Hopper, Excavator, Spin

    # Initialize ALL camera variables with dummy cameras immediately
    front_camera = cv2.VideoCapture()
    back_camera = cv2.VideoCapture()
    hopper_camera = cv2.VideoCapture()
    cameras = [front_camera, back_camera, hopper_camera]  # Track all cameras

    # Detect available cameras
    available_cams = detect_v4l_cameras()
    if len(available_cams) < 3:
        print(f"Warning: Only {len(available_cams)} cameras detected!")
        # Fallback to traditional indexing
        available_cams = ['/dev/video0', '/dev/video1', '/dev/video2']

    # Open cameras with proper error handling
    camera_labels = ["Front Camera", "Back Camera", "Hopper Camera"]
    for i, dev in enumerate(available_cams[:3]):
        cap = open_camera(dev)
        if cap and cap.isOpened():
            # Release the dummy camera first
            cameras[i].release()
            # Assign the real camera
            cameras[i] = cap
            if i == 0:
                front_camera = cap
            elif i == 1:
                back_camera = cap
            elif i == 2:
                hopper_camera = cap
            print(f"Opened {camera_labels[i]} at {dev}")
        else:
            print(f"Failed to open {camera_labels[i]} at {dev}")
            # Keep the dummy camera as fallback
    
    # Boolean flag to control streaming on/off
    cameras_active = False

    # GUI Layout
    main_frame = tk.Frame(app)
    main_frame.pack(fill="both", expand=True)

    # Top Half: Front and Back camera feeds
    top_frame = tk.Frame(main_frame)
    top_frame.pack(fill="both", expand=True)

    # Front Camera Preview
    front_cam_frame = tk.Frame(top_frame, padx=5, pady=5, bg="white", relief=tk.SUNKEN, borderwidth=2)
    tk.Label(front_cam_frame, text="Front Camera", font=("Consolas", 15), bg="white").pack()
    front_camera_label = tk.Label(front_cam_frame, bg="black")
    front_camera_label.pack()
    front_cam_frame.pack(side="left", fill="both", expand=True, padx=10, pady=10)

    # Back camera preview
    back_cam_frame = tk.Frame(top_frame, padx=5, pady=5, bg="white", relief=tk.SUNKEN, borderwidth=2)
    tk.Label(back_cam_frame, text="Back Camera", font=("Consolas", 15), bg="white").pack()
    back_camera_label = tk.Label(back_cam_frame, bg="black")
    back_camera_label.pack()
    back_cam_frame.pack(side="left", fill="both", expand=True, padx=10, pady=10)

    # Bottom Half: Hopper Cam & Sliders 
    bottom_frame = tk.Frame(main_frame)
    bottom_frame.pack(fill="both", expand=True)

    # Hopper camera preview
    hopper_cam_frame = tk.Frame(bottom_frame, padx=5, pady=5, bg="white", relief=tk.SUNKEN, borderwidth=2)
    tk.Label(hopper_cam_frame, text="Hopper Camera", font=("Consolas", 15), bg="white").pack()
    hopper_camera_label = tk.Label(hopper_cam_frame, bg="black")
    hopper_camera_label.pack()
    hopper_cam_frame.pack(side="left", fill="both", expand=True, padx=10, pady=10)

    # Sliders panel
    sliders_frame = tk.Frame(bottom_frame)
    sliders_frame.pack(side="left", fill="both", expand=True, padx=10, pady=10)

    # Create all 3 actuator sliders
    hopper_slider_var, hopper_slider_widget = create_actuator_slider(sliders_frame, "Hopper Actuator", 0, motor_values, app)
    hopper_slider_widget.pack(fill="x", pady=5)

    excavation_slider_var, excavation_slider_widget = create_actuator_slider(sliders_frame, "Excavation Actuator", 1, motor_values, app)
    excavation_slider_widget.pack(fill="x", pady=5)

    spin_slider_var, spin_slider_widget = create_actuator_slider(sliders_frame, "Excavation Spin", 2, motor_values, app)    
    spin_slider_widget.pack(fill="x", pady=5)

    def update_camera_feeds():
        # Updates all camera feeds with error handling for disconnections
        if not cameras_active:
            return

        # Front camera update
        if front_camera and front_camera.isOpened():
            ret, frame = front_camera.read()
            if ret:
                img = cv2.resize(frame, (640, 360))
                img = ImageTk.PhotoImage(image=Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB)))
                front_camera_label.config(image=img)
                front_camera_label.img = img  # Keep reference
            else:
                front_camera_label.config(text="Front Camera Disconnected")
        else:
            front_camera_label.config(text="Front Camera Not Available")

        # Back camera update
        if back_camera and back_camera.isOpened():
            ret, frame = back_camera.read()
            if ret:
                img = cv2.resize(frame, (640, 360))
                img = ImageTk.PhotoImage(image=Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB)))
                back_camera_label.config(image=img)
                back_camera_label.img = img
            else:
                back_camera_label.config(text="Back Camera Disconnected")
        else:
            back_camera_label.config(text="Back Camera Not Available")

        # Hopper camera update
        if hopper_camera and hopper_camera.isOpened():
            ret, frame = hopper_camera.read()
            if ret:
                img = cv2.resize(frame, (512, 288))  # Slightly smaller
                img = ImageTk.PhotoImage(image=Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB)))
                hopper_camera_label.config(image=img)
                hopper_camera_label.img = img
            else:
                hopper_camera_label.config(text="Hopper Camera Disconnected")
        else:
            hopper_camera_label.config(text="Hopper Camera Not Available")

        app.after(100, update_camera_feeds)  # Continue updating

    def start_camera_streams():
        # Starts all camera streams with error recovery
        nonlocal cameras_active, front_camera, back_camera, hopper_camera
        if not cameras_active:
            # Attempt to reopen any disconnected cameras
            for i, cam in enumerate([front_camera, back_camera, hopper_camera]):
                if cam and not cam.isOpened() and i < len(available_cams):
                    new_cam = open_camera(available_cams[i])
                    if new_cam:
                        if i == 0: front_camera = new_cam
                        elif i == 1: back_camera = new_cam
                        else: hopper_camera = new_cam
            cameras_active = True
            update_camera_feeds()
        
    def stop_camera_streams():
        # Stops all camera streams and clears displays
        nonlocal cameras_active
        cameras_active = False
        for label in [front_camera_label, back_camera_label, hopper_camera_label]:
            label.config(image='', text='Camera Stream Stopped')

    def handle_closing():
        # Cleanup function when window closes
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
