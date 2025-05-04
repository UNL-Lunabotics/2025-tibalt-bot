import tkinter as tk
import cv2
from PIL import Image, ImageTk

# Updates the displayed percentage value when the slider is moved.
#   var: Variable linked to the slider
#   label: Label displaying the current percentage value
def update_label(var, label):
    # Update the label text with the current slider value followed by a percentage sign
    label.config(text=f"{var.get()}%")
    
# Creates a slider with a name and live-updating percentage value 
#   parent: The parent widget to contain the slider
#   name: The name of the slider (used as the label text)
def create_slider(parent, name):
    # Create a frame to hold one slider and its labels
    slider_frame = tk.Frame(parent, padx=10, pady=10)
    
    # Label displaying the name of the slider (e.g., "Hopper Actuator")
    label_name = tk.Label(slider_frame, text=name, font=("Arial", 12))
    label_name.pack()
    
    # Variable to store the current value of the slider (from 0 to 100)
    var = tk.IntVar()
    
    # Create the slider (horizontal scale), with a range from 0 to 100
    slider = tk.Scale(
        slider_frame,
        from_=0,
        to=100,
        orient="horizontal",  # Horizontal orientation
        length=200,  # Length of the slider in pixels
        width=15,  # Width of the slider's track
        variable=var  # Link the slider to the variable `var`
    )
    slider.pack()
    
    # Label to show the current slider value (initially 0%)
    label_value = tk.Label(slider_frame, text="0%", font=("Arial", 12))
    label_value.pack()
    
    # Trace changes to the slider variable `var`, update the label with the new value
    var.trace_add("write", lambda *args: update_label(var, label_value))
    
    # Return the variable and frame so they can be used later
    return var, slider_frame

# Main content frame of the application
def main(args = None):
    # Create the main GUI window
    app = tk.Tk()
    app.title('Bot Control')  # Title of the window
    app.minsize(1200, 800)  # Set minimum size of the window
    
    # Define cleanup function when closing the app (stopping cameras, releasing resources)
    def on_closing():
        stop_cameras()  # Stop any camera feeds
        front_cam.release()  # Release the front camera
        back_cam.release()  # Release the back camera
        hopper_cam.release()  # Release the hopper camera
        app.destroy()  # Close the app window
        
    # Assign the on_closing function to run when the window is closed
    app.protocol("WM_DELETE_WINDOW", on_closing)
    
    # Initialize the three camera outputs (using the first camera for all)
    front_cam = cv2.VideoCapture(0)  # Open front camera (camera 0)
    back_cam = front_cam  # Temporarily set back camera to front camera
    hopper_cam = front_cam  # Temporarily set hopper camera to front camera
    
    # Set the resolution (width and height) for each video feed
    width = 320
    height = 240
    for cam in (front_cam, back_cam, hopper_cam):
        cam.set(cv2.CAP_PROP_FRAME_WIDTH, width)  # Set width
        cam.set(cv2.CAP_PROP_FRAME_HEIGHT, height)  # Set height
        
    # Flag to indicate whether cameras are running
    camera_running = False
        
    # Container for camera frames (to hold all camera-related widgets)
    camera_container = tk.Frame(app, padx=10, pady=10, bg ="gray90", relief=tk.RIDGE, borderwidth=2)
    camera_container.pack(fill="both", expand=True, padx=10, pady=10)
        
    # Top Row Frame to hold front and back camera feeds
    top_frame = tk.Frame(camera_container, bg="gray90")
    top_frame.pack(fill="both", expand=True)
    
    # Front Camera Block
    front_frame = tk.Frame(top_frame, padx=5, pady=5, bg="white", relief=tk.SUNKEN, borderwidth=2)
    front_label = tk.Label(front_frame, text="Front Cam", font=("Arial", 14), bg="white")
    front_label.pack()  # Pack the front camera label
    front_cam_display = tk.Label(front_frame, bg="black")  # Label to show front camera feed
    front_cam_display.pack()
    front_frame.pack(side="left", padx=10, pady=10, fill="both", expand=True)
    
    # Back Camera Block (currently using the same feed as front)
    back_frame = tk.Frame(top_frame, padx=5, pady=5, bg="white", relief=tk.SUNKEN, borderwidth=2)
    back_label = tk.Label(back_frame, text="Back Cam", font=("Arial", 14), bg="white")
    back_label.pack()  # Pack the back camera label
    back_cam_display = tk.Label(back_frame, bg="black")  # Label to show back camera feed
    back_cam_display.pack()
    back_frame.pack(side="left", padx=10, pady=10, fill="both", expand=True)
    
    # Bottom Row Frame to hold the hopper camera feed
    bottom_frame = tk.Frame(camera_container, bg="gray90")
    bottom_frame.pack(fill="both", expand=True)
    
    # Hopper Camera Block
    hopper_frame = tk.Frame(bottom_frame, padx=5, pady=5, bg="white", relief=tk.SUNKEN, borderwidth=2)
    hopper_label = tk.Label(hopper_frame, text="Hopper Cam", font=("Arial", 14), bg="white")
    hopper_label.pack()  # Pack the hopper camera label
    hopper_cam_display = tk.Label(hopper_frame, bg="black")  # Label to show hopper camera feed
    hopper_cam_display.pack()
    hopper_frame.pack(padx=10, pady=10, fill="both", expand=True)
    
    # Function to update camera frames in the window
    def update_frames():
        if not camera_running:  # If cameras are not running, exit the function
            return

        # Update front camera feed
        ret1, frame1 = front_cam.read()  # Read frame from front camera
        if ret1:  # If successful, update the image
            image1 = cv2.cvtColor(frame1, cv2.COLOR_BGR2RGBA)  # Convert to RGBA format
            photo1 = ImageTk.PhotoImage(image=Image.fromarray(image1))  # Convert to ImageTk format
            front_cam_display.photo_image = photo1  # Keep a reference to the image
            front_cam_display.configure(image=photo1)  # Display the image

        # Update back camera feed (same as front camera here)
        ret2, frame2 = back_cam.read()
        if ret2:
            image2 = cv2.cvtColor(frame2, cv2.COLOR_BGR2RGBA)
            photo2 = ImageTk.PhotoImage(image=Image.fromarray(image2))
            back_cam_display.photo_image = photo2
            back_cam_display.configure(image=photo2)

        # Update hopper camera feed
        ret3, frame3 = hopper_cam.read()
        if ret3:
            image3 = cv2.cvtColor(frame3, cv2.COLOR_BGR2RGBA)
            photo3 = ImageTk.PhotoImage(image=Image.fromarray(image3))
            hopper_cam_display.photo_image = photo3
            hopper_cam_display.configure(image=photo3)

        app.after(10, update_frames)  # Continuously update every 10 milliseconds

    # Start the camera feeds
    def start_cameras():
        nonlocal camera_running
        if not camera_running:
            camera_running = True
            update_frames()  # Start updating the frames

    # Stop the camera feeds
    def stop_cameras():
        nonlocal camera_running
        camera_running = False
        # Clear the displays
        front_cam_display.configure(image='')
        back_cam_display.configure(image='')
        hopper_cam_display.configure(image='')
    
    # Create and pack the label widget for the video
    label_widget = tk.Label(app)
    label_widget.pack()
    
    # Set up sliders at the bottom of the window
    sliders_frame = tk.Frame(app, padx=10, pady=10)
    sliders_frame.pack(side="bottom", fill="x")
    
    # Create sliders for hopper actuator, excavation actuator, and excavation spin
    hopper_acutator_slider, hopper_actuator_frame = create_slider(sliders_frame, "Hopper Actuator")
    excavation_actuator_slider, excavation_actuator_frame = create_slider(sliders_frame, "Excavation Actuator")
    excavation_spin_slider, excavation_spin_frame = create_slider(sliders_frame, "Excavation Spin")
    
    # Pack the sliders into the frame
    hopper_actuator_frame.pack(side="left", expand=True)
    excavation_actuator_frame.pack(side="left", expand=True)
    excavation_spin_frame.pack(side="left", expand=True)
    
    # Start the cameras
    start_cameras()
    
    # Start the GUI event loop
    app.mainloop()

# Entrypoint when run as a standalone script
if __name__ == '__main__':
    main()