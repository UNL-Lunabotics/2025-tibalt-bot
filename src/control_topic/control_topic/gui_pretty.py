import tkinter as tk
from tkinter import ttk
import cv2
from PIL import Image, ImageTk

class ModernSlider(ttk.Frame):
    def __init__(self, parent, label_text, index, motor_values, app):
        super().__init__(parent, padding=(10, 5))
        
        # Configure grid layout
        self.columnconfigure(0, weight=1)
        self.rowconfigure(1, weight=1)
        
        # Label
        self.label = ttk.Label(self, text=label_text, font=('Segoe UI', 11, 'bold'))
        self.label.grid(row=0, column=0, sticky='w', pady=(0, 5))
        
        # Slider
        self.slider_var = tk.IntVar()
        self.slider = ttk.Scale(
            self,
            from_=0,
            to=100,
            orient='horizontal',
            variable=self.slider_var,
            command=lambda v: self._update_entry()
        )
        self.slider.grid(row=1, column=0, sticky='ew', padx=(0, 10))
        
        # Entry frame
        entry_frame = ttk.Frame(self)
        entry_frame.grid(row=1, column=1, sticky='e')
        
        # Entry widget
        self.entry = ttk.Entry(entry_frame, width=5, font=('Segoe UI', 10), justify='center')
        self.entry.pack(side='left')
        
        # Percentage label
        ttk.Label(entry_frame, text="%").pack(side='left', padx=(2, 0))
        
        # Initialize entry
        self.entry.insert(0, "0")
        
        # Bindings
        self.entry.bind('<Return>', lambda e: self._update_slider())
        self.slider_var.trace_add('write', lambda *args: self._update_motor_values(index, motor_values))
        
        # Store reference to app for focus management
        self.app = app
    
    def _update_entry(self):
        """Update the entry widget when slider changes"""
        self.entry.delete(0, tk.END)
        self.entry.insert(0, str(self.slider_var.get()))
        self.app.focus_set()  # Remove focus from entry
    
    def _update_slider(self):
        """Update the slider when entry changes"""
        try:
            value = int(self.entry.get())
            value = max(0, min(100, value))
            self.slider_var.set(value)
        except ValueError:
            self.entry.delete(0, tk.END)
            self.entry.insert(0, str(self.slider_var.get()))
        self.app.focus_set()  # Remove focus from entry
    
    def _update_motor_values(self, index, motor_values):
        """Update the motor values array"""
        motor_values[index] = self.slider_var.get()

class CameraFrame(ttk.LabelFrame):
    def __init__(self, parent, title, camera):
        super().__init__(parent, text=title, padding=(5, 5, 5, 5))
        self.camera = camera
        self.label = ttk.Label(self, background='black')
        self.label.pack(fill='both', expand=True)
        self.scale = 1.0  # Default scale

class TibaltControlPanel:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title('Tibalt Control Panel')
        self.root.geometry('1200x800')
        self.root.minsize(1000, 700)
        
        # Style configuration
        self._configure_styles()
        
        # Motor values storage
        self.motor_values = [0, 0, 0]  # Hopper, Excavator, Spin
        
        # Initialize cameras
        self.front_camera = cv2.VideoCapture(0)
        self.back_camera = cv2.VideoCapture(1)
        self.hopper_camera = cv2.VideoCapture(2)
        
        # Set camera resolutions
        self.base_width, self.base_height = 640, 360
        self._configure_cameras()
        
        # Camera streaming state
        self.cameras_active = False
        
        # Build UI
        self._create_widgets()
        
        # Start camera streams
        self.start_camera_streams()
        
        # Handle window closing
        self.root.protocol("WM_DELETE_WINDOW", self.handle_closing)
        
        # Start main loop
        self.root.mainloop()
    
    def _configure_styles(self):
        """Configure modern styling for widgets"""
        style = ttk.Style(self.root)
        
        # Use 'vista' theme if available (Windows), otherwise 'clam'
        if 'vista' in style.theme_names():
            style.theme_use('vista')
        else:
            style.theme_use('clam')
        
        # Customize styles
        style.configure('TFrame', background='#f0f0f0')
        style.configure('TLabel', background='#f0f0f0')
        style.configure('TLabelFrame', background='#f0f0f0', bordercolor='#ccc', borderwidth=2)
        style.configure('TScale', troughcolor='#e0e0e0')
    
    def _configure_cameras(self):
        """Configure camera settings"""
        for camera in [self.front_camera, self.back_camera, self.hopper_camera]:
            if camera.isOpened():
                camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.base_width)
                camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.base_height)
    
    def _create_widgets(self):
        """Create and arrange all widgets"""
        # Main container
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill='both', expand=True, padx=10, pady=10)
        
        # Top section - camera views
        top_frame = ttk.Frame(main_frame)
        top_frame.pack(fill='both', expand=True, pady=(0, 10))
        
        # Camera frames
        self.front_cam_frame = CameraFrame(top_frame, "Front Camera", self.front_camera)
        self.front_cam_frame.pack(side='left', fill='both', expand=True, padx=(0, 10))
        
        self.back_cam_frame = CameraFrame(top_frame, "Back Camera", self.back_camera)
        self.back_cam_frame.pack(side='left', fill='both', expand=True)
        
        # Bottom section - hopper cam and controls
        bottom_frame = ttk.Frame(main_frame)
        bottom_frame.pack(fill='both', expand=True)
        
        # Hopper camera
        self.hopper_cam_frame = CameraFrame(bottom_frame, "Hopper Camera", self.hopper_camera)
        self.hopper_cam_frame.pack(side='left', fill='both', expand=True, padx=(0, 10))
        
        # Control panel
        control_frame = ttk.LabelFrame(bottom_frame, text="Actuator Controls", padding=(15, 10))
        control_frame.pack(side='left', fill='both', expand=True)
        
        # Create sliders
        self.hopper_slider = ModernSlider(control_frame, "Hopper Actuator", 0, self.motor_values, self.root)
        self.hopper_slider.pack(fill='x', pady=5)
        
        self.excavation_slider = ModernSlider(control_frame, "Excavation Actuator", 1, self.motor_values, self.root)
        self.excavation_slider.pack(fill='x', pady=5)
        
        self.spin_slider = ModernSlider(control_frame, "Excavation Spin", 2, self.motor_values, self.root)
        self.spin_slider.pack(fill='x', pady=5)
        
        # Status bar
        self.status_bar = ttk.Frame(self.root, height=25, relief='sunken')
        self.status_bar.pack(fill='x', side='bottom')
        self.status_label = ttk.Label(self.status_bar, text="Ready", font=('Segoe UI', 9))
        self.status_label.pack(side='left', padx=5)
    
    def update_camera_feeds(self):
        """Update all camera feeds"""
        if not self.cameras_active:
            return
        
        # Front camera
        success_front, frame_front = self.front_camera.read()
        if success_front:
            resized_front = cv2.resize(frame_front, 
                                     (int(self.base_width * self.front_cam_frame.scale), 
                                      int(self.base_height * self.front_cam_frame.scale)))
            front_image = ImageTk.PhotoImage(image=Image.fromarray(cv2.cvtColor(resized_front, cv2.COLOR_BGR2RGB)))
            self.front_cam_frame.label.config(image=front_image)
            self.front_cam_frame.label.image = front_image
        else:
            self.front_cam_frame.label.config(text="Front Camera Not Available", foreground='red')
        
        # Back camera
        success_back, frame_back = self.back_camera.read()
        if success_back:
            resized_back = cv2.resize(frame_back, 
                                     (int(self.base_width * self.back_cam_frame.scale), 
                                      int(self.base_height * self.back_cam_frame.scale)))
            back_image = ImageTk.PhotoImage(image=Image.fromarray(cv2.cvtColor(resized_back, cv2.COLOR_BGR2RGB)))
            self.back_cam_frame.label.config(image=back_image)
            self.back_cam_frame.label.image = back_image
        else:
            self.back_cam_frame.label.config(text="Back Camera Not Available", foreground='red')
        
        # Hopper camera
        success_hopper, frame_hopper = self.hopper_camera.read()
        if success_hopper:
            resized_hopper = cv2.resize(frame_hopper, 
                                      (int(self.base_width * self.hopper_cam_frame.scale), 
                                       int(self.base_height * self.hopper_cam_frame.scale)))
            hopper_image = ImageTk.PhotoImage(image=Image.fromarray(cv2.cvtColor(resized_hopper, cv2.COLOR_BGR2RGB)))
            self.hopper_cam_frame.label.config(image=hopper_image)
            self.hopper_cam_frame.label.image = hopper_image
        else:
            self.hopper_cam_frame.label.config(text="Hopper Camera Not Available", foreground='red')
        
        # Schedule next update
        self.root.after(100, self.update_camera_feeds)
    
    def start_camera_streams(self):
        """Start all camera streams"""
        if not self.cameras_active:
            self.cameras_active = True
            self.update_camera_feeds()
            self.status_label.config(text="Camera streams active")
    
    def stop_camera_streams(self):
        """Stop all camera streams"""
        self.cameras_active = False
        for frame in [self.front_cam_frame, self.back_cam_frame, self.hopper_cam_frame]:
            frame.label.config(image='', text="Camera stream stopped", foreground='gray')
        self.status_label.config(text="Camera streams stopped")
    
    def handle_closing(self):
        """Handle window closing event"""
        self.stop_camera_streams()
        if self.front_camera.isOpened():
            self.front_camera.release()
        if self.back_camera.isOpened() and self.back_camera != self.front_camera:
            self.back_camera.release()
        if self.hopper_camera.isOpened() and self.hopper_camera != self.front_camera:
            self.hopper_camera.release()
        self.root.destroy()

if __name__ == '__main__':
    TibaltControlPanel()