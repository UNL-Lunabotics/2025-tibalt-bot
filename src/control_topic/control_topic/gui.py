import tkinter as tk
import cv2
from PIL import Image, ImageTk

def update_label(var, label):
    label.config(text=f"{var.get()}%")
    
def create_slider(parent, name):
    slider_frame = tk.Frame(parent, padx=10, pady=5)
    label_name = tk.Label(slider_frame, text=name, font=("Arial", 12))
    label_name.pack()
    
    var = tk.IntVar()
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
    
    label_value = tk.Label(slider_frame, text="0%", font=("Arial", 12))
    label_value.pack()
    var.trace_add("write", lambda *args: update_label(var, label_value))
    
    return var, slider_frame

def main(args=None):
    app = tk.Tk()
    app.title('Bot Control')
    app.minsize(1200, 800)
    
    def on_closing():
        stop_cameras()
        if front_cam.isOpened(): front_cam.release()
        if back_cam.isOpened() and back_cam != front_cam: back_cam.release()
        if hopper_cam.isOpened() and hopper_cam != front_cam: hopper_cam.release()
        app.destroy()
        
    app.protocol("WM_DELETE_WINDOW", on_closing)
    
    # Initialize cameras
    front_cam = cv2.VideoCapture(0)
    back_cam = cv2.VideoCapture(1) if cv2.VideoCapture(1).isOpened() else front_cam
    hopper_cam = cv2.VideoCapture(2) if cv2.VideoCapture(2).isOpened() else front_cam
    
    # Set base resolution (16:9)
    base_width, base_height = 640, 360
    for cam in {front_cam, back_cam, hopper_cam}:
        if cam.isOpened():
            cam.set(cv2.CAP_PROP_FRAME_WIDTH, base_width)
            cam.set(cv2.CAP_PROP_FRAME_HEIGHT, base_height)
        
    camera_running = False
    
    # Main container
    main_container = tk.Frame(app)
    main_container.pack(fill="both", expand=True)
    
    # Top container for front and back cameras
    top_container = tk.Frame(main_container)
    top_container.pack(fill="both", expand=True)
    
    # Front Camera
    front_frame = tk.Frame(top_container, padx=5, pady=5, bg="white", relief=tk.SUNKEN, borderwidth=2)
    tk.Label(front_frame, text="Front Cam", font=("Arial", 14), bg="white").pack()
    front_cam_display = tk.Label(front_frame, bg="black")
    front_cam_display.pack()
    front_frame.pack(side="left", fill="both", expand=True, padx=10, pady=10)
    
    # Back Camera
    back_frame = tk.Frame(top_container, padx=5, pady=5, bg="white", relief=tk.SUNKEN, borderwidth=2)
    tk.Label(back_frame, text="Back Cam", font=("Arial", 14), bg="white").pack()
    back_cam_display = tk.Label(back_frame, bg="black")
    back_cam_display.pack()
    back_frame.pack(side="left", fill="both", expand=True, padx=10, pady=10)
    
    # Bottom container for hopper cam and sliders
    bottom_container = tk.Frame(main_container)
    bottom_container.pack(fill="both", expand=True)
    
    # Hopper Camera (left side of bottom)
    hopper_frame = tk.Frame(bottom_container, padx=5, pady=5, bg="white", relief=tk.SUNKEN, borderwidth=2)
    tk.Label(hopper_frame, text="Hopper Cam", font=("Arial", 14), bg="white").pack()
    hopper_cam_display = tk.Label(hopper_frame, bg="black")
    hopper_cam_display.pack()
    hopper_frame.pack(side="left", fill="both", expand=True, padx=10, pady=10)
    
    # Sliders container (right side of bottom)
    sliders_container = tk.Frame(bottom_container)
    sliders_container.pack(side="left", fill="both", expand=True, padx=10, pady=10)
    
    # Sliders setup - stacked vertically
    hopper_var, hopper_slider_frame = create_slider(sliders_container, "Hopper Actuator")
    hopper_slider_frame.pack(fill="x", pady=5)
    
    excav_var, excav_slider_frame = create_slider(sliders_container, "Excavation Actuator")
    excav_slider_frame.pack(fill="x", pady=5)
    
    spin_var, spin_slider_frame = create_slider(sliders_container, "Excavation Spin")
    spin_slider_frame.pack(fill="x", pady=5)
    
    def update_frames():
        if not camera_running:
            return

        # Scaling factors
        front_scale = 1
        back_scale = 1
        hopper_scale = 0.8

        # Front camera
        ret1, frame1 = front_cam.read()
        if ret1:
            display_width = int(base_width * front_scale)
            display_height = int(base_height * front_scale)
            frame1 = cv2.resize(frame1, (display_width, display_height), 
                              interpolation=cv2.INTER_LINEAR)
            photo1 = ImageTk.PhotoImage(image=Image.fromarray(cv2.cvtColor(frame1, cv2.COLOR_BGR2RGB)))
            front_cam_display.config(image=photo1)
            front_cam_display.photo_image = photo1
        else:
            front_cam_display.config(text="Front Camera Not Available")

        # Back camera
        ret2, frame2 = back_cam.read()
        if ret2:
            display_width = int(base_width * back_scale)
            display_height = int(base_height * back_scale)
            frame2 = cv2.resize(frame2, (display_width, display_height),
                              interpolation=cv2.INTER_LINEAR)
            photo2 = ImageTk.PhotoImage(image=Image.fromarray(cv2.cvtColor(frame2, cv2.COLOR_BGR2RGB)))
            back_cam_display.config(image=photo2)
            back_cam_display.photo_image = photo2
        else:
            back_cam_display.config(text="Back Camera Not Available")

        # Hopper camera
        ret3, frame3 = hopper_cam.read()
        if ret3:
            display_width = int(base_width * hopper_scale)
            display_height = int(base_height * hopper_scale)
            frame3 = cv2.resize(frame3, (display_width, display_height),
                              interpolation=cv2.INTER_LINEAR)
            photo3 = ImageTk.PhotoImage(image=Image.fromarray(cv2.cvtColor(frame3, cv2.COLOR_BGR2RGB)))
            hopper_cam_display.config(image=photo3)
            hopper_cam_display.photo_image = photo3
        else:
            hopper_cam_display.config(text="Hopper Camera Not Available")

        app.after(10, update_frames)

    def start_cameras():
        nonlocal camera_running
        if not camera_running:
            camera_running = True
            update_frames()

    def stop_cameras():
        nonlocal camera_running
        camera_running = False
        front_cam_display.config(image='')
        back_cam_display.config(image='')
        hopper_cam_display.config(image='')
    
    start_cameras()
    app.mainloop()

if __name__ == '__main__':
    main()