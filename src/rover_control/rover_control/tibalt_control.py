# This is the Python ROS2 library
import rclpy
from rclpy.qos import QoSProfile
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

# This is the Python ROS library that communicates with the joystick controller
from sensor_msgs.msg import Joy

# The RoboClaw library we use to control the motors
from std_msgs.msg import Int16 #RoboClaw requires Int16 types
from std_msgs.msg import Int16MultiArray

'''CONSTANT DECLARATION'''
# Motor speed constants to help make the code more readable
# They range from 0-127, with 0 as full power backward, 64 as stop, and 127 as full power forwarded
MOTOR_FORWARDS = 127
MOTOR_BACKWARDS = 0
MOTOR_STOP = 64

# Servo constants to help make the code more readable
# It ranges from 0-180 degrees
SERVO_FULL_CLOSE = 0
SERVO_FULL_OPEN = 180

'''CONTROL SCHEME''' #TODO: add in button indices
EXCAV_DIG_B = 0
EXCAV_RETRACT_B = 0
EXCAV_EXTEND_B = 0
HOPPER_LATCH_B = 0
HOPPER_RETRACT_B = 0
HOPPER_EXTEND_B = 0

class LINEAR_ACTUATOR_STATE(enumerate):
    '''Enum to keep track of the current hopper state'''
    RESTING = 0
    EXTENDING = 1
    RETRACTING = 2


class Tibalt(Node):
    '''The node class for all the main Tibalt logic. This contains the functionality
    for drive train and autonomy'''

    def __init__(self):
        super().__init__('tibalt') # Initializes this as a node with the name 'tibalt'
        # Hopper class variables
        self.hopper_state = LINEAR_ACTUATOR_STATE.RESTING
        self.hopper_actuator_motor = MOTOR_STOP
        self.hopper_latch_servo = SERVO_FULL_CLOSE
        self.hopper_is_latched = True
        
        # Drivetrain class variables
        self.dt_left_motor = MOTOR_STOP
        self.dt_right_motor = MOTOR_STOP

        # Excavation motor variables
        self.excav_state = LINEAR_ACTUATOR_STATE.RESTING
        self.excav_dig_motor = MOTOR_STOP
        self.excav_actuator_motor = MOTOR_STOP
        self.excav_is_digging = False

        # This creates a Rate object that we will use to sleep the system at the specified frequency (in Hz)
        # We need this to make sure the node's don't publish data too quickly
        self.rate = self.create_rate(
            frequency=10,
            clock=self.get_clock()
        )

        # This node will publish the motor speeds we want the Arduino to set things to
        self.publisher = self.create_publisher(
            msg_type=Int16MultiArray,
            topic='motor_speeds',
            qos_profile=QoSProfile(depth=10)
        )

        start_speeds = Int16MultiArray()
        # Drivetrain left, drivetrain right, excavation dig, excavation actuator, hopper actuator, hopper latch
        start_speeds.data = [0, 0, MOTOR_STOP, MOTOR_STOP, MOTOR_STOP, SERVO_FULL_CLOSE]
        self.publisher.publish(start_speeds)

        # This node subscribes to 'joystick_data' (published from joystick_pipeline)
        # This Joy object gives the current axis and button input from the joystick
        self.subsciption = self.create_subscription(
            msg_type=Joy,
            topic='joystick_data',
            callback=self.control_callback, # This function gets called when message received
            qos_profile=QoSProfile(depth=10)
        )

    # Logic for computing drivetrain motor speeds
    def drivetrain_logic(self, x_axis, y_axis) -> None:
        # Drivetrain motors go from -100 to 100 with 0 being resting
            # The drivetrain motors are weird so we have to use the servo library in arduino code
        left_motor = y_axis + x_axis
        right_motor = y_axis - x_axis
        max_mag = max(abs(left_motor), abs(right_motor))
        if max_mag > 1.0:
            left_motor /= max_mag
            right_motor /= max_mag
        self.dt_left_motor = int(left_motor * 100.0)
        self.dt_right_motor = int(right_motor * 100.0)

    # Logic for computing excavation motor speeds
    def excavation_logic(self, linear_actuator_extend, linear_actuator_retract, dig) -> None:
        # Excavation linear actuator logic
        if linear_actuator_extend:
            self.excav_actuator_motor = MOTOR_FORWARDS
        elif linear_actuator_retract:
            self.excav_actuator_motor = MOTOR_BACKWARDS
        else:
            self.excav_actuator_motor = MOTOR_STOP

        # Excavation dig motor logic
        if dig:
            self.excav_dig_motor = MOTOR_FORWARDS
        else:
            self.excav_dig_motor = MOTOR_STOP

    def hopper_logic(self, linear_actuator_extend, linear_actuator_retract, servo_latch) -> None:
        # Hopper linear actuator logic
        if linear_actuator_extend:
            self.hopper_actuator_motor = MOTOR_FORWARDS
        elif linear_actuator_retract:
            self.hopper_actuator_motor = MOTOR_BACKWARDS
        else:
            self.hopper_actuator_motor = MOTOR_STOP

        # Flip the hopper latch is button is pressed
        if servo_latch:
            self.hopper_is_latched = not self.hopper_is_latched

        if self.hopper_is_latched:
            self.hopper_latch_servo = SERVO_FULL_CLOSE
        else:
            self.hopper_latch_servo = SERVO_FULL_OPEN
        

    # Calls functions for each section of the rover when we receive new topic data
    def control_callback(self, joystick: Joy) -> None:
        self.drivetrain_logic(
            self=self, 
            x_axis=joystick.axes[0], 
            y_axis=joystick.axes[1]
        )
        self.excavation_logic(
            self=self, 
            linear_actuator_extend=joystick.buttons[EXCAV_EXTEND_B],
            linear_actuator_retract=joystick.buttons[EXCAV_RETRACT_B],
            dig=joystick.buttons[EXCAV_DIG_B]
        )
        self.hopper_logic(
            self=self,
            linear_actuator_extend=joystick.buttons[HOPPER_EXTEND_B],
            linear_actuator_retract=joystick.buttons[HOPPER_RETRACT_B],
            servo_latch=joystick.buttons[HOPPER_LATCH_B]
        )

        # Put all of the motor values in an array and publish them
        motor_speeds = Int16MultiArray()
        motor_speeds.data = [
            self.dt_left_motor,
            self.dt_right_motor,
            self.excav_dig_motor,
            self.excav_actuator_motor,
            self.hopper_actuator_motor,
            self.hopper_latch_servo
        ]
        self.publisher.publish(motor_speeds)


def main(args=None) -> None:
    """Initializes Tibalt and starts the control loop"""

    try:
        rclpy.init(args=args)
        tibalt = Tibalt()

        while rclpy.ok(): # Ensures that the code runs continuously until shutdown
            
            # This will allow the node to process pending callback requests once before
            # continuing to run this loop. This allows us to control the callback rate
            rclpy.spin_once(tibalt)

            # Sleep the node for 10 Hz without blocking the entire system
            tibalt.loop_rate.sleep()
    except (KeyboardInterrupt, ExternalShutdownException):
        # Shuts down if a KeyboardInterrupt or ExternalShutdownException is detected
        # i.e. if Ctrl+C is pressed or if ROS2 is shutdown externally
        
        # This command cleans up ROS2 resources to shutdown gracefully
        rclpy.shutdown()

if __name__ == '__main__':
    main()
