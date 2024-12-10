#!/usr/bin/env python


# rosrun user_input text_command_publisher.py "command"

import rospy
from std_msgs.msg import String
import sys

def publish_text():
    # Initialize the ROS Node
    rospy.init_node('text_publisher_node', anonymous=True)
    
    # Create a Publisher object, publishing to the 'text_topic' topic
    publisher = rospy.Publisher('user_input/speech', String, queue_size=10)
    
    # Set the loop rate
    rate = rospy.Rate(10)  # 10 Hz

    # Check if the script received command-line arguments
    if len(sys.argv) < 2:
        rospy.loginfo("Please provide text input as an argument.")
        return

    # Extract the text input from command-line arguments
    # text_input = ' '.join(sys.argv[1:])
    text_input = "pick up then place the pringles back on the table"
    i = 0
    while not rospy.is_shutdown():
        # Publish the text input
        rospy.loginfo(f"Publishing: {text_input}")
        publisher.publish(text_input)
        
        # Sleep to maintain the loop rate
        rate.sleep()
        i += 1
        if i == 10:
            break

if __name__ == '__main__':
    try:
        publish_text()
    except rospy.ROSInterruptException:
        pass
