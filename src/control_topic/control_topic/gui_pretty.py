import tkinter as tk
from tkinter import ttk
import cv2
from PIL import Image, ImageTk
import numpy as np
import pyrealsense2 as rs
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt

class AppConfig:
    # Modern color scheme
    COLORS = {
        'primary': '#2c3e50',
        'secondary': '#34495e',
        'accent': '#3498db',
        'background': '#ecf0f1',
        'text': '#2c3e50',
        'success': '#2ecc71',
        'warning': '#f39c12',
        'danger': '#e74c3c'
    }
    
    # Font settings
    FONTS = {
        'title': ('Segoe UI', 16, 'bold'),
        'subtitle': ('Segoe UI', 12),
        'body': ('Segoe UI', 10),
        'small': ('Segoe UI', 8)
    }
    
    # Camera configuration
    CAMERA_NAMES = {
        'front': "Front Camera",
        'back': "Rear Camera", 
        'hopper': "Hopper Monitoring"
    }
    
    CAMERA_SIZES = {
        'front': (640, 360),
        'back': (640, 360),
        'hopper': (512, 288)
    }
    
    # Control settings
    SLIDER_CONFIGS = [
        {"name": "Hopper Actuator", "unit": "%", "min": 0, "max": 100},
        {"name": "Excavation Arm", "unit": "°", "min": 0, "max": 180},
        {"name": "Rotation Speed", "unit": "rpm", "min": 0, "max": 60}
    ]

class ModernCameraBase:
    def __init__(self):
        self.connected = False
        self.last_update_time = 0
        self.frame_rate = 0
        
    def release(self):
        if self.connected:
            if hasattr(self, 'cap'):
                self.cap.release()
            if hasattr(self, 'pipeline'):
                self.pipeline.stop()
        self.connected = False
        
    def get_frame_rate(self):
        return f"{self.frame_rate:.1f} FPS" if self.connected else "Offline"

class ModernWebcam(ModernCameraBase):
    def __init__(self, camera_number):
        super().__init__()
        try:
            self.cap = cv2.VideoCapture(camera_number)
            self.connected = self.cap.isOpened()
            
            if self.connected:
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
                self.last_frame_time = cv2.getTickCount()
                
        except Exception as e:
            print(f"Webcam error: {e}")
            self.connected = False
    
    def read(self):
        if not self.connected:
            return False, None
            
        ret, frame = self.cap.read()
        
        # Calculate frame rate
        now = cv2.getTickCount()
        time_elapsed = (now - self.last_frame_time) / cv2.getTickFrequency()
        self.frame_rate = 1.0 / time_elapsed if time_elapsed > 0 else 0
        self.last_frame_time = now
        
        if ret and len(frame.shape) == 2:
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
            
        return ret, frame

class ModernRealSenseCamera(ModernCameraBase):
    def __init__(self):
        super().__init__()
        self.pipeline = None
        self.config = None
        self.last_frame_time = 0
        
    def _initialize(self):
        try:
            self.pipeline = rs.pipeline()
            self.config = rs.config()
            
            width, height = AppConfig.CAMERA_SIZES['front']
            
            self.config.enable_stream(
                rs.stream.color,
                width, height,
                rs.format.bgr8,
                30
            )
            
            self.pipeline.start(self.config)
            self.connected = True
            self.last_frame_time = cv2.getTickCount()
            
        except Exception as e:
            print(f"RealSense error: {e}")
            self.connected = False
    
    def read(self):
        if not self.connected:
            self._initialize()
            if not self.connected:
                return False, None
        
        try:
            frames = self.pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            
            # Calculate frame rate
            now = cv2.getTickCount()
            time_elapsed = (now - self.last_frame_time) / cv2.getTickFrequency()
            self.frame_rate = 1.0 / time_elapsed if time_elapsed > 0 else 0
            self.last_frame_time = now
            
            if not color_frame:
                return False, None
                
            return True, np.asanyarray(color_frame.get_data())
            
        except Exception as e:
            print(f"RealSense read error: {e}")
            return False, None

class ModernControlPanel:
    def __init__(self, root):
        self.root = root
        self.root.title('Tibalt Control Panel v2.0')
        self.root.geometry('1280x800')
        self.root.minsize(1024, 768)
        self.running = True
        
        # Initialize theme state
        self.dark_mode = False
        
        # Initialize controls state
        self.slider_values = [0] * len(AppConfig.SLIDER_CONFIGS)
        self.system_status = "OK"
        self.battery_level = 100
        
        # Configure styles
        self._configure_styles()
        
        # Initialize cameras
        self.cameras = {
            'front': {
                'camera': ModernRealSenseCamera(),
                'panel': None,
                'status': None
            },
            'back': {
                'camera': ModernRealSenseCamera(),
                'panel': None,
                'status': None
            },
            'hopper': {
                'camera': ModernWebcam(0),
                'panel': None,
                'status': None
            }
        }
        
        # Create placeholder images
        self.placeholders = self._create_modern_placeholders()
        
        # Build UI
        self._build_ui()
        
        # Apply initial theme
        self._toggle_theme()

    def _configure_styles(self):
        style = ttk.Style()
        style.theme_use('clam')
        
        # Configure button styles
        style.configure('TButton', font=AppConfig.FONTS['body'])
        style.configure('Accent.TButton', 
                       background=AppConfig.COLORS['accent'],
                       foreground='white')
        
        # Configure frame styles
        style.configure('TFrame', background=AppConfig.COLORS['background'])
        style.configure('Card.TFrame', 
                       background='white',
                       relief='raised',
                       borderwidth=1)
        
        # Configure label styles
        style.configure('Title.TLabel',
                       font=AppConfig.FONTS['title'],
                       foreground=AppConfig.COLORS['primary'])
        style.configure('Subtitle.TLabel',
                       font=AppConfig.FONTS['subtitle'],
                       foreground=AppConfig.COLORS['secondary'])

    def _create_modern_placeholders(self):
        placeholders = {}
        
        for cam_key in ['front', 'back', 'hopper']:
            width, height = AppConfig.CAMERA_SIZES[cam_key]
            img = np.full((height, width, 3), 40 if self.dark_mode else 230, dtype=np.uint8)
            
            # Add camera name
            text = AppConfig.CAMERA_NAMES[cam_key]
            font = cv2.FONT_HERSHEY_SIMPLEX
            text_size = cv2.getTextSize(text, font, 0.8, 2)[0]
            text_x = (width - text_size[0]) // 2
            text_y = (height + text_size[1]) // 2 - 20
            cv2.putText(img, text, (text_x, text_y), font, 0.8, 
                       (100, 100, 255) if self.dark_mode else (50, 50, 200), 2)
            
            # Add status text
            status = "Camera Offline"
            status_size = cv2.getTextSize(status, font, 0.6, 1)[0]
            status_x = (width - status_size[0]) // 2
            status_y = text_y + 40
            cv2.putText(img, status, (status_x, status_y), font, 0.6, 
                       (200, 200, 200) if self.dark_mode else (100, 100, 100), 1)
            
            placeholders[cam_key] = ImageTk.PhotoImage(Image.fromarray(img))
            
        return placeholders

    def _build_ui(self):
        # Create main container
        self.main_container = ttk.Frame(self.root)
        self.main_container.pack(fill='both', expand=True, padx=10, pady=10)
        
        # Header section
        self._build_header()
        
        # Main content area
        self.content_frame = ttk.Frame(self.main_container)
        self.content_frame.pack(fill='both', expand=True, pady=(10, 0))
        
        # Camera section
        self._build_camera_section()
        
        # Control section
        self._build_control_section()
        
        # Status section
        self._build_status_section()

    def _build_header(self):
        header_frame = ttk.Frame(self.main_container)
        header_frame.pack(fill='x', pady=(0, 10))
        
        # Title
        title_frame = ttk.Frame(header_frame)
        title_frame.pack(side='left', fill='y')
        
        ttk.Label(title_frame, text="TIBALT", style='Title.TLabel').pack(anchor='w')
        ttk.Label(title_frame, text="Robotic Excavation System", style='Subtitle.TLabel').pack(anchor='w')
        
        # Controls
        control_frame = ttk.Frame(header_frame)
        control_frame.pack(side='right')
        
        self.theme_btn = ttk.Button(control_frame, text="Dark Mode", 
                                   command=self._toggle_theme, style='Accent.TButton')
        self.theme_btn.pack(side='left', padx=5)
        
        ttk.Button(control_frame, text="Emergency Stop", 
                  command=self._emergency_stop).pack(side='left', padx=5)

    def _build_camera_section(self):
        camera_frame = ttk.Frame(self.content_frame)
        camera_frame.pack(fill='both', expand=True)
        
        # Main cameras (front and back)
        main_cam_frame = ttk.Frame(camera_frame)
        main_cam_frame.pack(side='left', fill='both', expand=True)
        
        for cam_key in ['front', 'back']:
            cam_card = ttk.Frame(main_cam_frame, style='Card.TFrame')
            cam_card.pack(side='left', fill='both', expand=True, padx=5, pady=5)
            
            # Header
            header = ttk.Frame(cam_card)
            header.pack(fill='x', padx=5, pady=5)
            
            ttk.Label(header, text=AppConfig.CAMERA_NAMES[cam_key], 
                     style='Subtitle.TLabel').pack(side='left')
            
            status_label = ttk.Label(header, text="Offline", style='Subtitle.TLabel')
            status_label.pack(side='right')
            self.cameras[cam_key]['status'] = status_label
            
            # Camera display
            display = ttk.Label(cam_card)
            display.pack(padx=5, pady=5)
            self.cameras[cam_key]['panel'] = display
        
        # Hopper camera and controls
        right_frame = ttk.Frame(camera_frame)
        right_frame.pack(side='left', fill='y')
        
        # Hopper camera
        hopper_card = ttk.Frame(right_frame, style='Card.TFrame')
        hopper_card.pack(fill='both', expand=True, padx=5, pady=5)
        
        header = ttk.Frame(hopper_card)
        header.pack(fill='x', padx=5, pady=5)
        
        ttk.Label(header, text=AppConfig.CAMERA_NAMES['hopper'], 
                 style='Subtitle.TLabel').pack(side='left')
        
        status_label = ttk.Label(header, text="Offline", style='Subtitle.TLabel')
        status_label.pack(side='right')
        self.cameras['hopper']['status'] = status_label
        
        display = ttk.Label(hopper_card)
        display.pack(padx=5, pady=5)
        self.cameras['hopper']['panel'] = display

    def _build_control_section(self):
        control_frame = ttk.Frame(self.content_frame)
        control_frame.pack(fill='x', pady=(10, 0))
        
        # Create a notebook (tabbed interface)
        control_notebook = ttk.Notebook(control_frame)
        control_notebook.pack(fill='both', expand=True)
        
        # Manual Control Tab
        manual_tab = ttk.Frame(control_notebook)
        control_notebook.add(manual_tab, text="Manual Control")
        
        self._build_manual_controls(manual_tab)
        
        # Autonomous Tab
        auto_tab = ttk.Frame(control_notebook)
        control_notebook.add(auto_tab, text="Autonomous")
        
        self._build_autonomous_controls(auto_tab)

    def _build_manual_controls(self, parent):
        # Create a frame for each control
        for i, config in enumerate(AppConfig.SLIDER_CONFIGS):
            control_card = ttk.Frame(parent, style='Card.TFrame')
            control_card.pack(fill='x', padx=5, pady=5)
            
            # Control name and value display
            header = ttk.Frame(control_card)
            header.pack(fill='x', padx=10, pady=(10, 5))
            
            ttk.Label(header, text=config['name'], style='Subtitle.TLabel').pack(side='left')
            
            value_var = tk.StringVar(value=f"0 {config['unit']}")
            ttk.Label(header, textvariable=value_var, style='Subtitle.TLabel').pack(side='right')
            
            # Slider control
            slider_frame = ttk.Frame(control_card)
            slider_frame.pack(fill='x', padx=10, pady=(0, 10))
            
            slider = ttk.Scale(
                slider_frame,
                from_=config['min'],
                to=config['max'],
                orient='horizontal',
                command=lambda v, i=i, var=value_var, cfg=config: self._update_slider_value(v, i, var, cfg)
            )
            slider.pack(fill='x', expand=True)
            
            # Min/Max labels
            limits_frame = ttk.Frame(slider_frame)
            limits_frame.pack(fill='x')
            
            ttk.Label(limits_frame, text=f"{config['min']} {config['unit']}").pack(side='left')
            ttk.Label(limits_frame, text=f"{config['max']} {config['unit']}").pack(side='right')

    def _build_autonomous_controls(self, parent):
        # Placeholder for autonomous controls
        info_frame = ttk.Frame(parent, style='Card.TFrame')
        info_frame.pack(fill='both', expand=True, padx=5, pady=5)
        
        ttk.Label(info_frame, text="Autonomous Operation Mode", 
                 style='Subtitle.TLabel').pack(pady=20)
        
        # Mission parameters
        param_frame = ttk.Frame(info_frame)
        param_frame.pack(fill='x', padx=20, pady=10)
        
        ttk.Label(param_frame, text="Excavation Depth:").grid(row=0, column=0, sticky='e')
        depth_entry = ttk.Entry(param_frame)
        depth_entry.grid(row=0, column=1, padx=5, pady=2)
        ttk.Label(param_frame, text="cm").grid(row=0, column=2, sticky='w')
        
        ttk.Label(param_frame, text="Excavation Area:").grid(row=1, column=0, sticky='e')
        area_entry = ttk.Entry(param_frame)
        area_entry.grid(row=1, column=1, padx=5, pady=2)
        ttk.Label(param_frame, text="m²").grid(row=1, column=2, sticky='w')
        
        # Action buttons
        btn_frame = ttk.Frame(info_frame)
        btn_frame.pack(fill='x', pady=20)
        
        ttk.Button(btn_frame, text="Start Mission", 
                  style='Accent.TButton').pack(side='left', padx=10)
        ttk.Button(btn_frame, text="Pause Mission").pack(side='left', padx=10)
        ttk.Button(btn_frame, text="Return to Base").pack(side='right', padx=10)

    def _build_status_section(self):
        status_frame = ttk.Frame(self.main_container, style='Card.TFrame')
        status_frame.pack(fill='x', pady=(10, 0))
        
        # System status indicators
        ttk.Label(status_frame, text="System Status", 
                 style='Subtitle.TLabel').pack(anchor='w', padx=10, pady=5)
        
        indicators_frame = ttk.Frame(status_frame)
        indicators_frame.pack(fill='x', padx=10, pady=(0, 10))
        
        # Battery indicator
        batt_frame = ttk.Frame(indicators_frame)
        batt_frame.pack(side='left', padx=10)
        
        self.batt_canvas = tk.Canvas(batt_frame, width=150, height=30, highlightthickness=0)
        self.batt_canvas.pack()
        self._draw_battery(100)
        
        ttk.Label(batt_frame, text="Battery Level").pack()
        
        # System status
        status_frame = ttk.Frame(indicators_frame)
        status_frame.pack(side='left', padx=10)
        
        self.status_indicator = tk.Canvas(status_frame, width=30, height=30, highlightthickness=0)
        self.status_indicator.pack()
        self._draw_status_indicator("OK")
        
        ttk.Label(status_frame, text="System Status").pack()
        
        # Frame rate display
        fps_frame = ttk.Frame(indicators_frame)
        fps_frame.pack(side='right', padx=10)
        
        ttk.Label(fps_frame, text="Camera FPS").pack()
        
        self.fps_labels = {
            'front': ttk.Label(fps_frame, text="Front: Offline"),
            'back': ttk.Label(fps_frame, text="Back: Offline"),
            'hopper': ttk.Label(fps_frame, text="Hopper: Offline")
        }
        
        for label in self.fps_labels.values():
            label.pack(anchor='e')

    def _draw_battery(self, level):
        self.batt_canvas.delete("all")
        
        # Battery outline
        self.batt_canvas.create_rectangle(5, 5, 145, 25, outline='#333', width=2)
        self.batt_canvas.create_rectangle(145, 10, 150, 20, fill='#333', outline='#333')
        
        # Battery level
        if level > 70:
            color = AppConfig.COLORS['success']
        elif level > 30:
            color = AppConfig.COLORS['warning']
        else:
            color = AppConfig.COLORS['danger']
        
        width = 138 * (level / 100)
        self.batt_canvas.create_rectangle(8, 8, 8 + width, 22, fill=color, outline='')
        
        # Text
        self.batt_canvas.create_text(75, 15, text=f"{level}%", fill='white')

    def _draw_status_indicator(self, status):
        self.status_indicator.delete("all")
        
        if status == "OK":
            color = AppConfig.COLORS['success']
        elif status == "WARNING":
            color = AppConfig.COLORS['warning']
        else:
            color = AppConfig.COLORS['danger']
        
        self.status_indicator.create_oval(5, 5, 25, 25, fill=color, outline='')

    def _update_slider_value(self, value, index, var, config):
        try:
            val = round(float(value))
            self.slider_values[index] = val
            var.set(f"{val} {config['unit']}")
        except ValueError:
            pass

    def _toggle_theme(self):
        self.dark_mode = not self.dark_mode
        
        if self.dark_mode:
            # Dark theme colors
            bg = '#121212'
            fg = '#ffffff'
            card_bg = '#1e1e1e'
            self.theme_btn.config(text="Light Mode")
        else:
            # Light theme colors
            bg = AppConfig.COLORS['background']
            fg = AppConfig.COLORS['text']
            card_bg = 'white'
            self.theme_btn.config(text="Dark Mode")
        
        # Update main background
        self.main_container.config(style='TFrame')
        ttk.Style().configure('TFrame', background=bg)
        
        # Update card backgrounds
        ttk.Style().configure('Card.TFrame', 
                            background=card_bg,
                            bordercolor='#ccc' if not self.dark_mode else '#444')
        
        # Update text colors
        ttk.Style().configure('Title.TLabel', foreground=fg)
        ttk.Style().configure('Subtitle.TLabel', foreground=fg)
        
        # Recreate placeholders with new theme
        self.placeholders = self._create_modern_placeholders()
        
        # Redraw status indicators
        self._draw_battery(self.battery_level)
        self._draw_status_indicator(self.system_status)

    def _emergency_stop(self):
        self.system_status = "STOPPED"
        self._draw_status_indicator(self.system_status)
        
        # Flash the screen red
        self.main_container.config(style='Emergency.TFrame')
        ttk.Style().configure('Emergency.TFrame', background=AppConfig.COLORS['danger'])
        self.root.after(300, lambda: self.main_container.config(style='TFrame'))

    def update_camera_views(self):
        if not self.running:
            return
        
        for cam_key, cam_data in self.cameras.items():
            camera = cam_data['camera']
            panel = cam_data['panel']
            status = cam_data['status']
            
            if camera.connected:
                success, frame = camera.read()
                
                if success:
                    # Resize if needed
                    if cam_key == 'hopper':
                        frame = cv2.resize(frame, AppConfig.CAMERA_SIZES['hopper'])
                    
                    # Convert to PhotoImage
                    img = ImageTk.PhotoImage(Image.fromarray(frame))
                    
                    # Update display
                    panel.config(image=img)
                    panel.image = img
                    
                    # Update status
                    status.config(text="Active", foreground=AppConfig.COLORS['success'])
                    self.fps_labels[cam_key].config(text=f"{cam_key.capitalize()}: {camera.get_frame_rate()}")
                    continue
            
            # Show placeholder if camera not available
            panel.config(image=self.placeholders[cam_key])
            panel.image = self.placeholders[cam_key]
            
            # Update status
            status.config(text="Offline", foreground=AppConfig.COLORS['danger'])
            self.fps_labels[cam_key].config(text=f"{cam_key.capitalize()}: Offline")
        
        # Update battery level (simulate discharge)
        self.battery_level = max(0, self.battery_level - 0.1)
        self._draw_battery(self.battery_level)
        
        # Schedule next update
        self.root.after(30, self.update_camera_views)

    def on_close(self):
        self.running = False
        for cam_data in self.cameras.values():
            cam_data['camera'].release()
        self.root.destroy()
        
def main():
    root = tk.Tk()
    app = ModernControlPanel(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    app.update_camera_views()
    root.mainloop()

if __name__ == '__main__':
    main()