import rospy
import json
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool

class EmergencyStop:
    '''
    Setup emergency stop for the wireless controller to halt the agents' control commands.
    Toggles the actuator control commands ON or OFF.
    '''
    def __init__(self, path_to_config, DEBOUNCE_DURATION=0.5):
        # Load rostopic names
        with open(path_to_config + '/rostopics.json', 'r') as rostopics_file:
            rostopics = json.load(rostopics_file)

        self.pub = rospy.Publisher(rostopics['e_stop'], Bool, queue_size=10)
        self.sub = rospy.Subscriber(rostopics['joy'], Joy, self.joy_callback)

        self.is_estop_active = False
        self.last_toggle_time = rospy.Time.now()
        self.DEBOUNCE_DURATION = rospy.Duration(DEBOUNCE_DURATION) # [ms] debounce delay

    def joy_callback(self, data):
        button_state = data.buttons[0]  # Controller X button state
        current_time = rospy.Time.now()

        # Toggle only if button is pressed and debounce time has passed
        if button_state and ((current_time - self.last_toggle_time) > self.DEBOUNCE_DURATION):
            self.last_toggle_time = current_time
            self.is_estop_active = not self.is_estop_active

            self.pub.publish(Bool(data=self.is_estop_active))
            rospy.loginfo(f"Emergency stop active: {self.is_estop_active}")

if __name__ == "__main__":
    try:
        EmergencyStop()
    except rospy.ROSInterruptException:
        pass
