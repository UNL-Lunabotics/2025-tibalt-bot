# TODO: update using code from last year
# TODO: Testing code & Better comments (im bad at descriptions) 
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Bool
import tkinter as tk
import cv2
from PIL import Image, ImageTk
    
# Prototype GUI
def main(args = None):
    # rclpy.init(args = args)
    
    # Creates the main GUI window
    app = tk.Tk()
    app.title('Motor Control')
    # Window will close when escape key pressed
    app.bind('<Escape>', lambda e: app.quit())
    
    # # Gui Publishing node
    # node = Node('motor_shutdown')
    
    # # Creates a publisher for shutdown commands on the 'motor_shutdown' topic
    # # Queue size 10 ensures commands aren't lost if system is busy
    # publisher = node.create_publisher(Bool, 'motor_shutdown', 10)
        
    # # Callback for shutdown button; Publishes shutdown command
    # def shutdown_motor():
    #     msg = Bool()
    #     msg.data = True # True signles shutdown command
    #     publisher.publish(msg)
    #     node.get_logger().info('Published motor shutdown command')
    
    # Sets up live video
    vid = cv2.VideoCapture(0)
    width, height = 800, 600
    vid.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    vid.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    
    # Tracks the camera's State
    camera_running = False
    # Holds the ID for the after() loop
    camera_loop_id = None
    
  # Recursive function to output camera data to GUI
    def update_frame():
        nonlocal camera_loop_id
        if not camera_running:
            return
        # Captures the vidoe frame by frame
        ret, frame = vid.read()
        if not ret:
            return
        # Convert image from BGR (OpenCV) to RGBA (Tkinter Compatible)
        opencv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGBA)
        # Capture the latest frame and transform to Pillow image
        captured_image = Image.fromarray(opencv_image)
        # Convert captured image to Tkinter-compatible PhotoImage
        photo_image = ImageTk.PhotoImage(image=captured_image)
        # Displays the PhotoImage in the label
        label_widget.PhotoImage = photo_image
        label_widget.configure(image=photo_image)
        # Repeats the same process every 10ms (customizable)
        label_widget.after(10, update_frame)
    
    # Starts the video stream and updates the GUI
    def start_camera():
        nonlocal camera_running
        if not camera_running:
            camera_running = True
            # Hides Start button and shows Stop button
            start_video_button.pack_forget()
            stop_video_button.pack(side='left', padx=15)
            update_frame()
        
    # Stops the video stream and clears the display
    def stop_camera():
        nonlocal camera_running, camera_loop_id
        if(camera_running):
            camera_running = False
            if camera_loop_id:
                label_widget.after_cancel(camera_loop_id)
                camera_loop_id = None
            # Hides Stop Button and shows Start button
            stop_video_button.pack_forget()
            start_video_button.pack(side='left', padx=15)
            # Clears the image on the GUI
            label_widget.configure(image='')

# Sets up the entire GUI
    # Button Container
    button_frame = tk.Frame(app)
    button_frame.pack(padx=10)
    
    # Creates the shutdown button
    shutdown_button = tk.Button(
        app,
        text = 'Shutdown Motor',
        width= 15,
        height = 2,
        bg = 'red',
        fg = 'black'
        # command = shutdown_motor
    )
    shutdown_button.pack()
    
    # Creates the label on which the video will be displayed
    label_widget = tk.Label(app)
    label_widget.pack()
    
    # Creates a button which starts the live video.
    start_video_button = tk.Button(
        app,
        text = 'Start Live Video',
        width = 15,
        height = 2,
        bg = 'green',
        fg = 'black',
        command = start_camera
    )
    start_video_button.pack(side = 'left', padx=10)
    
    # Creates a button which stops the live video. 
    # This button is hidden until start button is pressed.
    stop_video_button = tk.Button(
        app,
        text = 'Stop Live Video',
        width = 15,
        height = 2, 
        bg = 'orange',
        fg = 'black',
        command = stop_camera
    )
    
    # Starts the GUI Event Loop
    app.mainloop()

    # # Cleanup when window closes
    # node.destroy_node()
    # rclpy.shutdown()
    vid.release()
    
# Entrypoint when run as a standalone script
if __name__ == '__main__':
    main()