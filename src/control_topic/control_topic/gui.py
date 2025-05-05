import tkinter as tk
import cv2
from PIL import Image, ImageTk
import numpy as np
import pyrealsense2 as rs

def update_label(slider_value_var, value_label):
    value_label.config(text=f"{slider_value_var.get()}%")

def create_slider(parent, name):
    frame = tk.Frame(parent)
    tk.Label(frame, text=name, font=("Arial", 12)).pack(anchor='w')
    
    # Slider and input field
    control_frame = tk.Frame(frame)
    
    slider_value_var = tk.IntVar()
    slider = tk.Scale(
        control_frame,
        from_=0,
        to=100,
        orient="horizontal",
        variable=slider_value_var,
        length=250,
        width=20,
        sliderlength=30,
        font=("Arial", 10)
    )
    slider.pack(side='left', padx=5)
    
    input_frame = tk.Frame(control_frame)
    
    # Entry widget with validation
    entry_var = tk.StringVar()
    entry = tk.Entry(
        input_frame,
        width=5,
        textvariable=entry_var,
        font=("Arial", 10),
        validate="key",
        validatecommand=(parent.register(lambda text: text.isdigit() or text == ""), '%P')
    )
    entry.pack(side='top')
    tk.Label(input_frame, text="%", font=("Arial", 10)).pack(side='top')
    
    def on_enter(event):
        try:
            value = int(entry_var.get())
            if 0 <= value <= 100:
                slider_value_var.set(value)
        except ValueError:
            pass  # Invalid input, do nothing
    
    entry.bind('<Return>', on_enter)
    
    input_frame.pack(side='left', padx=5)
    control_frame.pack()
    
    value_label = tk.Label(frame, text="0%", font=("Arial", 12))
    value_label.pack()
    
    slider_value_var.trace_add("write", lambda *_: update_label(slider_value_var, value_label))
    return slider_value_var, frame

def create_placeholder(width, height, text):
    image = np.zeros((height, width, 3), dtype=np.uint8)
    image[:] = (50, 50, 50)
    cv2.putText(image, text, 
               ((width - cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.8, 2)[0][0]) // 2,
               (height + cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.8, 2)[0][1]) // 2),
               cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
    return ImageTk.PhotoImage(Image.fromarray(image))

class CameraBase:
    def release(self):
        if hasattr(self, 'connected') and self.connected:
            getattr(self, 'cap', None) and self.cap.release()
            getattr(self, 'pipeline', None) and self.pipeline.stop()

class ArducamCamera(CameraBase):
    def __init__(self, index):
        self.cap = cv2.VideoCapture(index)
        self.connected = self.cap.isOpened()
        if self.connected:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    def read(self):
        if not self.connected: return False, None
        ret, frame = self.cap.read()
        if ret and len(frame.shape) == 2:
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        return ret, frame

class RealSenseCamera(CameraBase):
    def __init__(self):
        self.connected = False  # Don't initialize yet
        self.pipeline = None
        
    def ensure_connected(self):
        if not self.connected and not self.pipeline:
            try:
                self.pipeline = rs.pipeline()
                self.config = rs.config()
                self.config.enable_stream(rs.stream.color, 640, 360, rs.format.bgr8, 30)
                self.pipeline.start(self.config)
                self.connected = True
            except:
                self.connected = False

    def read(self):
        self.ensure_connected()  # Initialize on first read
        if not self.connected: return False, None
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        return color_frame, np.asanyarray(color_frame.get_data()) if color_frame else (False, None)

def main():
    app = tk.Tk()
    app.title('Tibalt Control Panel')
    app.minsize(1200, 800)
    
    front_cam = RealSenseCamera()
    back_cam = RealSenseCamera()
    hopper_cam = ArducamCamera(0)
    
    placeholders = {
        'front': create_placeholder(640, 360, "Front Camera Not Available"),
        'back': create_placeholder(640, 360, "Back Camera Not Available"),
        'hopper': create_placeholder(512, 288, "Hopper Camera Not Available")
    }
    
    def create_camera_frame(parent, title):
        frame = tk.Frame(parent, relief=tk.SUNKEN, borderwidth=2)
        tk.Label(frame, text=title).pack()
        label = tk.Label(frame)
        label.pack()
        return frame, label
    
    main_frame = tk.Frame(app)
    main_frame.pack(fill='both', expand=True)
    
    cam_display = tk.Frame(main_frame)
    cam_display.pack(fill='both', expand=True)
    
    front_frame, front_label = create_camera_frame(cam_display, "Front Cam")
    front_frame.pack(side='left', fill='both', expand=True, padx=10, pady=10)
    
    back_frame, back_label = create_camera_frame(cam_display, "Back Cam")
    back_frame.pack(side='left', fill='both', expand=True, padx=10, pady=10)
    
    hopper_slider_frame = tk.Frame(main_frame)
    hopper_slider_frame.pack(fill='both', expand=True)
    
    hopper_frame, hopper_label = create_camera_frame(hopper_slider_frame, "Hopper Cam")
    hopper_frame.pack(side='left', fill='both', expand=True, padx=10, pady=10)
    
    sliders_frame = tk.Frame(hopper_slider_frame)
    sliders_frame.pack(side='left', fill='both', expand=True, padx=10, pady=10)
    
    # Create sliders
    slider_names = ["Hopper Actuator", "Excavation Actuator", "Excavation Spin"]
    
    for name in slider_names:
        _, frame = create_slider(sliders_frame, name)
        frame.pack(fill='x', pady=10)
    
    running = True
    
    def update():
        if not running: return
        
        for cam, label, placeholder in [
            (front_cam, front_label, placeholders['front']),
            (back_cam, back_label, placeholders['back']),
            (hopper_cam, hopper_label, placeholders['hopper'])
        ]:
            if cam and cam.connected:
                success, frame = cam.read()
                if success:
                    if cam == hopper_cam: frame = cv2.resize(frame, (512, 288))
                    img = ImageTk.PhotoImage(Image.fromarray(frame))
                    label.config(image=img)
                    label.image = img
                    continue
            label.config(image=placeholder)
            label.image = placeholder
        
        app.after(10, update)
    
    def on_close():
        nonlocal running
        running = False
        for cam in [front_cam, back_cam, hopper_cam]:
            if cam: cam.release()
        app.destroy()
    
    app.protocol("WM_DELETE_WINDOW", on_close)
    update()
    app.mainloop()

if __name__ == '__main__':
    main()