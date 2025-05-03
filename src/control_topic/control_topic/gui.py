# TODO: update using code from last year
# TODO: Testing code & Better comments (im bad at descriptions) 
import tkinter as tk
import cv2
from PIL import Image, ImageTk

# Updates the displayed percentage value when the slider is moved.
#   var: Variable linked to the slider
#   label: Label displaying the current percentage value
def update_label(var, label):
    label.config(text=f"{var.get()}%")
    
# Creates a slider with a name and live-updating percentage value 
def create_slider(parent, name):\
    
    # Creates a frame to hold one slider and it's labels
    slider_frame = tk.Frame(parent, padx=10, pady=10)
    
    # Label to dispay slider's name
    label_name = tk.Label(slider_frame, text=name, font=("Arial", 12))
    label_name.pack()
    
    # Variable for the slider's value
    var = tk.IntVar()
    
    # Horizontal slider (Scale)
    slider = tk.Scale(
        slider_frame,
        from_=0,
        to=100,
        orient="horizontal",
        length=200,
        width=15,
        variable=var
    )
    slider.pack()
    
    # Label to show current value
    label_value = tk.Label(slider_frame, text="0%", font=("Arial", 12))
    label_value.pack()
    
    # Link value changes to table
    var.trace_add("write", lambda *args: update_label(var, label_value))
    
    return var, slider_frame

# Main content frame
def main(args = None):
    
    # Creates the main GUI window
    app = tk.Tk()
    app.title('Bot Control')
    app.minsize(1200, 800)
    
    # Initializes the three camera outputs
    front_cam = cv2.VideoCapture(0)
    back_cam = cv2.VideoCapture(0)
    hopper_cam = cv2.VideoCapture(0)
    
    # Sets the width and height of each video
    width = 320
    height = 240
    for cam in (front_cam, back_cam, hopper_cam):
        cam.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cam.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        
    camera_running = False
        
    # Sets up the camera frames (holds everything camera-related)
    camera_container = tk.Frame(app, padx=10, pady=10, bg ="gray90", relief=tk.RIDGE, borderwidth=2)
    camera_container.pack(fill="both", expand=True, padx=10, pady=10)
        
    # Top Row Frame
    top_frame = tk.Frame(camera_container, bg="gray90")
    top_frame.pack(fill="both", expand=True)
    
    # Front Came Block
    front_frame = tk.Frame(top_frame, padx=5, pady=5, bg="white", relief=tk.SUNKEN, borderwidth=2)
    front_label = tk.Label(front_frame, text="Front Cam", font=("Arial", 14), bg="white")
    front_label.pack()
    front_cam_display = tk.Label(front_frame, bg="black")
    front_cam_display.pack()
    front_frame.pack(side="left", padx=10, pady=10, fill="both", expand=True)
    
    # Back Came Block
    back_frame = tk.Frame(top_frame, padx=5, pady=5, bg="white", relief=tk.SUNKEN, borderwidth=2)
    back_label = tk.Label(back_frame, text="Back Cam", font=("Arial", 14), bg="white")
    back_label.pack()
    back_cam_display = tk.Label(back_frame, bg="black")
    back_cam_display.pack()
    back_frame.pack(side="left", padx=10, pady=10, fill="both", expand=True)
    
    # Bottom Row Frame
    bottom_frame = tk.Frame(camera_container, bg="gray90")
    bottom_frame.pack(fill="both", expand=True)
    
    # Hopper Came Block
    hopper_frame = tk.Frame(bottom_frame, padx=5, pady=5, bg="white", relief=tk.SUNKEN, borderwidth=2)
    hopper_label = tk.Label(hopper_frame, text="Hopper Cam", font=("Arial", 14), bg="white")
    hopper_label.pack()
    hopper_cam_display = tk.Label(hopper_frame, bg="black")
    hopper_cam_display.pack()
    hopper_frame.pack(padx=10, pady=10, fill="both", expand=True)
    
    def update_frames():
        if not camera_running:
            return

        # Front Cam
        ret1, frame1 = front_cam.read()
        if ret1:
            image1 = cv2.cvtColor(frame1, cv2.COLOR_BGR2RGBA)
            photo1 = ImageTk.PhotoImage(image=Image.fromarray(image1))
            front_cam_display.photo_image = photo1
            front_cam_display.configure(image=photo1)

        # Back Cam
        ret2, frame2 = back_cam.read()
        if ret2:
            image2 = cv2.cvtColor(frame2, cv2.COLOR_BGR2RGBA)
            photo2 = ImageTk.PhotoImage(image=Image.fromarray(image2))
            back_cam_display.photo_image = photo2
            back_cam_display.configure(image=photo2)

        # Hopper Cam
        ret3, frame3 = hopper_cam.read()
        if ret3:
            image3 = cv2.cvtColor(frame3, cv2.COLOR_BGR2RGBA)
            photo3 = ImageTk.PhotoImage(image=Image.fromarray(image3))
            hopper_cam_display.photo_image = photo3
            hopper_cam_display.configure(image=photo3)

        app.after(10, update_frames)

    def start_cameras():
        nonlocal camera_running
        if not camera_running:
            camera_running = True
            update_frames()

    def stop_cameras():
        nonlocal camera_running
        camera_running = False
        # Clear displays
        front_cam_display.configure(image='')
        back_cam_display.configure(image='')
        hopper_cam_display.configure(image='')
    
# Sets up the entire GUI
    # Button Container
    button_frame = tk.Frame(app)
    button_frame.pack(padx=10)
    
    # Creates the label on which the video will be displayed
    label_widget = tk.Label(app)
    label_widget.pack()
    
    # Slider frame setup
    sliders_frame = tk.Frame(app, padx=10, pady=10)
    sliders_frame.pack(side="bottom", fill="x")
    
    hopper_acutator_slider, hopper_actuator_frame = create_slider(sliders_frame, "Hopper Actuator")
    excavation_actuator_slider, excavation_actuator_frame = create_slider(sliders_frame, "Excavation Actuator")
    excavation_spin_slider, excavation_spin_frame = create_slider(sliders_frame, "Excavation Spin")
    
    hopper_actuator_frame.pack(side="left", expand=True)
    excavation_actuator_frame.pack(side="left", expand=True)
    excavation_spin_frame.pack(side="left", expand=True)
    
    # Starts all the cameras
    start_cameras()
    
    # Starts the GUI Event Loop
    app.mainloop()

    # Cleanup when window closes
    front_cam.release()
    back_cam.release()
    hopper_cam.release()
    
# Entrypoint when run as a standalone script
if __name__ == '__main__':
    main()