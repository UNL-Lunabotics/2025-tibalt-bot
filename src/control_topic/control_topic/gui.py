# TODO: update using code from last year
# TODO: Testing code & Better comments (im bad at descriptions) 
import rclpy
from rclpy.node import Node
import tkinter as tk
from std_msgs.msg import Bool
    
# Prototype GUI, Currently only contains shutdown button
def main(args = None):
    rclpy.init(args = args)
    
    # Gui Publishing node
    node = Node('motor_shutdown')
    
    # Creates a publisher for shutdown commands on the 'motor_shutdown' topic
    # Queue size 10 ensures commands aren't lost if system is busy
    publisher = node.create_publisher(Bool, 'motor_shutdown', 10)
        
    # Creates the main GUI window
    root = tk.Tk()
    root.title('Motor Control')
    
    # Callback for shutdown button; Publishes shutdown command
    def shutdown_motor():
        msg = Bool()
        msg.data = True # True signles shutdown command
        publisher.publish(msg)
        node.get_logger().info('Published motor shutdown command')
    
    # Creates the shutdown button
    # TODO: Stylizing / Padding
    shutdown_button = tk.Button(
        root,
        text = 'Shutdown Motor'
        command = shutdown_motor
    )
    shutdown_button.pack()
    
    # Starts the GUI Event Loop
    root.mainloop()
    
    # Cleanup when window closes
    node.destroy_node()
    rclpy.shutdown()
    
# Entrypoint when run as a standalone script
if __name__ == '__main__':
    main()