import tkinter as tk
import cv2
from PIL import Image, ImageTk

# Updates the label showing the slider value and stores it in the shared list
def update_slider_label(slider_var, value_label, index, slider_values):
    value_label.config(text=f"{slider_var.get()}%")
    slider_values[index] = slider_var.get()

# Creates a labeled slider for actuator control
def create_actuator_slider(parent, label_text, index, slider_values):
    frame = tk.Frame(parent, padx=10, pady=5)

    # Label for the name of the actuator
    name_label = tk.Label(frame, text=label_text, font=("Arial", 12))
    name_label.pack()

    # Integer variable that stores the slider value
    slider_var = tk.IntVar()

    # Slider widget
    slider_widget = tk.Scale(
        frame,
        from_=0,
        to=100,
        orient="horizontal",
        length=200,
        width=15,
        variable=slider_var
    )
    slider_widget.pack()

    # Label to display the slider's current percentage value
    value_label = tk.Label(frame, text="0%", font=("Arial", 12))
    value_label.pack()

    # When the slider is moved, update the label and shared value list
    slider_var.trace_add("write", lambda *args: update_slider_label(slider_var, value_label, index, slider_values))

    return slider_var, frame

def main(args=None):
    # Create the main application window
    app = tk.Tk()
    app.title('Robot Control Panel')
    app.minsize(1200, 800)

    # Shared list to store current values of the 3 sliders
    slider_values = [0, 0, 0]  # [Hopper Actuator, Excavation Actuator, Excavation Spin]

    # Gracefully release resources and close the app
    def handle_closing():
        stop_camera_streams()
        if front_camera.isOpened(): front_camera.release()
        if back_camera.isOpened() and back_camera != front_camera: back_camera.release()
        if hopper_camera.isOpened() and hopper_camera != front_camera: hopper_camera.release()
        app.destroy()

    app.protocol("WM_DELETE_WINDOW", handle_closing)

    # Open camera devices by index
    front_camera = cv2.VideoCapture(0)
    back_camera = cv2.VideoCapture(1)
    hopper_camera = cv2.VideoCapture(2)

    # Set resolution to 640x360 (16:9 aspect ratio)
    base_width, base_height = 640, 360
    for camera in {front_camera, back_camera, hopper_camera}:
        if camera.isOpened():
            camera.set(cv2.CAP_PROP_FRAME_WIDTH, base_width)
            camera.set(cv2.CAP_PROP_FRAME_HEIGHT, base_height)

    cameras_active = False  # Flag to control camera streaming

    # ---- GUI Layout ----
    main_frame = tk.Frame(app)
    main_frame.pack(fill="both", expand=True)

    # ---- Top row for front and back cameras ----
    top_frame = tk.Frame(main_frame)
    top_frame.pack(fill="both", expand=True)

    # Front camera display
    front_cam_frame = tk.Frame(top_frame, padx=5, pady=5, bg="white", relief=tk.SUNKEN, borderwidth=2)
    tk.Label(front_cam_frame, text="Front Camera", font=("Arial", 14), bg="white").pack()
    front_camera_label = tk.Label(front_cam_frame, bg="black")
    front_camera_label.pack()
    front_cam_frame.pack(side="left", fill="both", expand=True, padx=10, pady=10)

    # Back camera display
    back_cam_frame = tk.Frame(top_frame, padx=5, pady=5, bg="white", relief=tk.SUNKEN, borderwidth=2)
    tk.Label(back_cam_frame, text="Back Camera", font=("Arial", 14), bg="white").pack()
    back_camera_label = tk.Label(back_cam_frame, bg="black")
    back_camera_label.pack()
    back_cam_frame.pack(side="left", fill="both", expand=True, padx=10, pady=10)

    # ---- Bottom row for hopper cam and sliders ----
    bottom_frame = tk.Frame(main_frame)
    bottom_frame.pack(fill="both", expand=True)

    # Hopper camera display
    hopper_cam_frame = tk.Frame(bottom_frame, padx=5, pady=5, bg="white", relief=tk.SUNKEN, borderwidth=2)
    tk.Label(hopper_cam_frame, text="Hopper Camera", font=("Arial", 14), bg="white").pack()
    hopper_camera_label = tk.Label(hopper_cam_frame, bg="black")
    hopper_camera_label.pack()
    hopper_cam_frame.pack(side="left", fill="both", expand=True, padx=10, pady=10)

    # Sliders container
    sliders_frame = tk.Frame(bottom_frame)
    sliders_frame.pack(side="left", fill="both", expand=True, padx=10, pady=10)

    # Create sliders for each actuator
    hopper_slider_var, hopper_slider_widget = create_actuator_slider(sliders_frame, "Hopper Actuator", 0, slider_values)
    hopper_slider_widget.pack(fill="x", pady=5)

    excavation_slider_var, excavation_slider_widget = create_actuator_slider(sliders_frame, "Excavation Actuator", 1, slider_values)
    excavation_slider_widget.pack(fill="x", pady=5)

    spin_slider_var, spin_slider_widget = create_actuator_slider(sliders_frame, "Excavation Spin", 2, slider_values)
    spin_slider_widget.pack(fill="x", pady=5)

    # Continuously read from the cameras and update the display
    def update_camera_feeds():
        if not cameras_active:
            return

        # Optional scale factors (can be adjusted if needed)
        front_scale = 1
        back_scale = 1
        hopper_scale = 0.8

        # Read and display front camera
        success_front, frame_front = front_camera.read()
        if success_front:
            resized_front = cv2.resize(frame_front, (int(base_width * front_scale), int(base_height * front_scale)))
            front_image = ImageTk.PhotoImage(image=Image.fromarray(cv2.cvtColor(resized_front, cv2.COLOR_BGR2RGB)))
            front_camera_label.config(image=front_image)
            front_camera_label.photo_image = front_image
        else:
            front_camera_label.config(text="Front Camera Not Available")

        # Read and display back camera
        success_back, frame_back = back_camera.read()
        if success_back:
            resized_back = cv2.resize(frame_back, (int(base_width * back_scale), int(base_height * back_scale)))
            back_image = ImageTk.PhotoImage(image=Image.fromarray(cv2.cvtColor(resized_back, cv2.COLOR_BGR2RGB)))
            back_camera_label.config(image=back_image)
            back_camera_label.photo_image = back_image
        else:
            back_camera_label.config(text="Back Camera Not Available")

        # Read and display hopper camera
        success_hopper, frame_hopper = hopper_camera.read()
        if success_hopper:
            resized_hopper = cv2.resize(frame_hopper, (int(base_width * hopper_scale), int(base_height * hopper_scale)))
            hopper_image = ImageTk.PhotoImage(image=Image.fromarray(cv2.cvtColor(resized_hopper, cv2.COLOR_BGR2RGB)))
            hopper_camera_label.config(image=hopper_image)
            hopper_camera_label.photo_image = hopper_image
        else:
            hopper_camera_label.config(text="Hopper Camera Not Available")

        # TODO: Integrate ROS2 publishing here
        # You can publish the values from slider_values list to a ROS2 topic
        # Example: ros_publisher.publish(slider_values)

        # Repeat this function every 100ms
        app.after(100, update_camera_feeds)

    # Start reading from the cameras
    def start_camera_streams():
        nonlocal cameras_active
        if not cameras_active:
            cameras_active = True
            update_camera_feeds()

    # Stop camera streaming and clear image previews
    def stop_camera_streams():
        nonlocal cameras_active
        cameras_active = False
        front_camera_label.config(image='')
        back_camera_label.config(image='')
        hopper_camera_label.config(image='')

    start_camera_streams()
    app.mainloop()

if __name__ == '__main__':
    main()
