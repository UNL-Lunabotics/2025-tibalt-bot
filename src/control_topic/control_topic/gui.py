import tkinter as tk
import cv2
from PIL import Image, ImageTk

# Updates the text label showing the current value of the slider
# Also stores the value in an array so it can be accessed elsewhere
def update_slider_label(slider_var, value_label, index, slider_values):
    value_label.config(text=f"{slider_var.get()}%")
    slider_values[index] = slider_var.get() 
    
# Creates a GUI slider with labels and an entry box to control an actuator
# Each slider is labeled, has a horizontal slider bar, a manual entry box, and a percentage label
def create_actuator_slider(parent, label_text, index, slider_values):
    frame = tk.Frame(parent, padx=10, pady=10) # Container for one actuator slider

    # Label describing what motor each slider controls
    name_label = tk.Label(frame, text=label_text, font=("Consolas", 13))
    name_label.pack(side="top")

    # Horizontal frame to place slider and entry side-by-side
    slider_entry_frame = tk.Frame(frame)
    slider_entry_frame.pack(side="top", pady=5)

    slider_var = tk.IntVar()  # tkinter variable used to track slider value (integer)

    # Slider bar itself, allowing values from 0 to 100
    slider_widget = tk.Scale(
        slider_entry_frame,
        from_=0,
        to=100,
        orient="horizontal",  # slider moves left to right
        length=400,           # pixel length of slider
        sliderlength=40,      # size of the slider handle
        width=20,             # thickness of the slider track
        variable=slider_var   # link this slider to our tkinter IntVar
    )
    slider_widget.pack(side="left")

    # Entry field that allows the user to manually enter a value
    entry = tk.Entry(slider_entry_frame, width=5, justify='center')
    entry.pack(side="left", padx=5)

    # Label for displaying the current percentage value of the slider
    value_label = tk.Label(frame, text="0%", font=("Consolas", 13))
    value_label.pack()

    # Updates motor value when the user presses ENTER inside manual input
    # Attempts to convert entry to an integer
    # Clamps value between 0 and 100
    # Sets slider to that value
    # If entry is not valid, reset entry to match slider.
    def on_entry_return(event=None):
        try:
            value = int(entry.get())
            value = max(0, min(100, value))
            slider_var.set(value)
        except ValueError:
            entry.delete(0, tk.END)
            entry.insert(0, str(slider_var.get()))
            
    # Triggered when ENTER is pressed
    entry.bind("<Return>", on_entry_return)

    # Updates motor value when slider is moved
    # Updates Text label
    # Updates manual entry box
    # Updates slider_values array
    def on_slider_change(*args):
        value_label.config(text=f"{slider_var.get()}%")
        entry.delete(0, tk.END)                    
        entry.insert(0, str(slider_var.get()))
        slider_values[index] = slider_var.get()       
        
    # Binds function to slider movement
    slider_var.trace_add("write", on_slider_change)  
    
    # Return the variable and the whole slider frame
    return slider_var, frame

# Main function that sets up the entire GUI application
def main(args=None):
    # Create main application window
    app = tk.Tk()
    app.title('Tibalt Control Panel')
    app.minsize(1200, 800)

    # List to keep track of all three actuator slider values
    # Index 0 = Hopper, 1 = Excavator, 2 = Spin
    slider_values = [0, 0, 0]

    # Called when the window is closed (X button clicked)
    # Stops cameras
    # Releases opened devices to clear resources
    # Closes window & Exits the program
    def handle_closing():
        stop_camera_streams()
        if front_camera.isOpened(): front_camera.release()
        if back_camera.isOpened() and back_camera != front_camera: back_camera.release()
        if hopper_camera.isOpened() and hopper_camera != front_camera: hopper_camera.release()
        app.destroy()

    # Binds the function to the window closing
    app.protocol("WM_DELETE_WINDOW", handle_closing)

    # Open the 3 cameras by index (0 = front, 1 = back, 2 = hopper)
    front_camera = cv2.VideoCapture(0)
    back_camera = cv2.VideoCapture(1)
    hopper_camera = cv2.VideoCapture(2)

    # Set all cameras to a resolution of 640x360 (16:9 ratio)
    base_width, base_height = 640, 360
    for camera in {front_camera, back_camera, hopper_camera}:
        if camera.isOpened():
            camera.set(cv2.CAP_PROP_FRAME_WIDTH, base_width)
            camera.set(cv2.CAP_PROP_FRAME_HEIGHT, base_height)

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
    tk.Label(front_cam_frame, text="Front Camera", font=("Consolas", 15
), bg="white").pack()
    front_camera_label = tk.Label(front_cam_frame, bg="black")
    front_camera_label.pack()
    front_cam_frame.pack(side="left", fill="both", expand=True, padx=10, pady=10)

    # Back camera preview
    back_cam_frame = tk.Frame(top_frame, padx=5, pady=5, bg="white", relief=tk.SUNKEN, borderwidth=2)
    tk.Label(back_cam_frame, text="Back Camera", font=("Consolas", 15
), bg="white").pack()
    back_camera_label = tk.Label(back_cam_frame, bg="black")
    back_camera_label.pack()
    back_cam_frame.pack(side="left", fill="both", expand=True, padx=10, pady=10)

    # Bottom Half: Hopper Cam & Sliders 
    bottom_frame = tk.Frame(main_frame)
    bottom_frame.pack(fill="both", expand=True)

    # Hopper camera preview
    hopper_cam_frame = tk.Frame(bottom_frame, padx=5, pady=5, bg="white", relief=tk.SUNKEN, borderwidth=2)
    tk.Label(hopper_cam_frame, text="Hopper Camera", font=("Consolas", 15
), bg="white").pack()
    hopper_camera_label = tk.Label(hopper_cam_frame, bg="black")
    hopper_camera_label.pack()
    hopper_cam_frame.pack(side="left", fill="both", expand=True, padx=10, pady=10)

    # Sliders panel
    sliders_frame = tk.Frame(bottom_frame)
    sliders_frame.pack(side="left", fill="both", expand=True, padx=10, pady=10)

    # Create all 3 actuator sliders and place them in the panel
    hopper_slider_var, hopper_slider_widget = create_actuator_slider(sliders_frame, "Hopper Actuator", 0, slider_values)
    hopper_slider_widget.pack(fill="x", pady=5)

    excavation_slider_var, excavation_slider_widget = create_actuator_slider(sliders_frame, "Excavation Actuator", 1, slider_values)
    excavation_slider_widget.pack(fill="x", pady=5)

    spin_slider_var, spin_slider_widget = create_actuator_slider(sliders_frame, "Excavation Spin", 2, slider_values)
    spin_slider_widget.pack(fill="x", pady=5)

    # Recursive function for updating camera feeds
    def update_camera_feeds():
        # Stop if streaming is disabled
        if not cameras_active:
            return 

        # Optional scaling (can shrink image if needed)
        front_scale = 1
        back_scale = 1
        hopper_scale = 0.8

        # Reads Front Camera
        success_front, frame_front = front_camera.read()
        if success_front:
            resized_front = cv2.resize(frame_front, (int(base_width * front_scale), int(base_height * front_scale)))
            front_image = ImageTk.PhotoImage(image=Image.fromarray(cv2.cvtColor(resized_front, cv2.COLOR_BGR2RGB)))
            front_camera_label.config(image=front_image)
            front_camera_label.photo_image = front_image
        else:
            front_camera_label.config(text="Front Camera Not Available")

        # Reads Back Camera
        success_back, frame_back = back_camera.read()
        if success_back:
            resized_back = cv2.resize(frame_back, (int(base_width * back_scale), int(base_height * back_scale)))
            back_image = ImageTk.PhotoImage(image=Image.fromarray(cv2.cvtColor(resized_back, cv2.COLOR_BGR2RGB)))
            back_camera_label.config(image=back_image)
            back_camera_label.photo_image = back_image
        else:
            back_camera_label.config(text="Back Camera Not Available")

        # Reads Hopper Camera
        success_hopper, frame_hopper = hopper_camera.read()
        if success_hopper:
            resized_hopper = cv2.resize(frame_hopper, (int(base_width * hopper_scale), int(base_height * hopper_scale)))
            hopper_image = ImageTk.PhotoImage(image=Image.fromarray(cv2.cvtColor(resized_hopper, cv2.COLOR_BGR2RGB)))
            hopper_camera_label.config(image=hopper_image)
            hopper_camera_label.photo_image = hopper_image
        else:
            hopper_camera_label.config(text="Hopper Camera Not Available")

        # TODO: Publish slider values via ROS2

        app.after(100, update_camera_feeds)  # Call this function again after 100ms

    # Starts the video streams from all cameras
    def start_camera_streams():
        nonlocal cameras_active
        if not cameras_active:
            cameras_active = True
            update_camera_feeds()

    # Stops camera streams and clears their preview areas
    def stop_camera_streams():
        nonlocal cameras_active
        cameras_active = False
        front_camera_label.config(image='')
        back_camera_label.config(image='')
        hopper_camera_label.config(image='')

    # Start showing camera previews
    start_camera_streams()
    # Start the tkinter GUI event loop
    app.mainloop()

# Run the main function when this script is executed directly
if __name__ == '__main__':
    main()
