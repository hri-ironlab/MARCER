#!/usr/bin/env python3
import time
import rospy
from std_msgs.msg import String
from alive_progress import alive_bar
import time
from std_msgs.msg import Float32

class SpeechToTextNode:
    def __init__(self, record_time):
        # Initialize the ROS node
        rospy.init_node("no_whisper_speech_to_text_node")

        self.progress_publisher = rospy.Publisher('/user_input/listening_progress', Float32, queue_size=10)

        self.key_phrase = "hey fetch"

        self.speech_publisher = rospy.Publisher('user_input/speech', String, queue_size=10)

        self.record_time = record_time
        self.progress = Float32()

        self.i = 1
        self.transcript = ""

    def user_input_handler(self, msg):
        self.transcript = msg
    
    def run(self):
        # Main functionaliy for real-time audio transcription
        try:
            print('Listening for "Hey Fetch [COMMAND]"...')
            with alive_bar(self.record_time, title='', spinner=None, monitor=False, stats=False, theme='classic') as progress_bar:
                for i in range(self.record_time):
                    self.progress.data = i / self.record_time
                    self.progress_publisher.publish(self.progress)
                    time.sleep(.5)
                    progress_bar()
        except KeyboardInterrupt:
            print("\nRecording interrupted by user.")


        if self.i == 1:
            self.transcript = "hey fetch pick the pringles and put them one the table"
        self.i+=1

        if self.key_phrase in self.transcript:
            task = self.transcript.split(self.key_phrase)[1]
            self.speech_publisher.publish(task)
            print("\nSending command:", task)
            self.transcript = ""
        print("\n")

if __name__ == "__main__":
    speech_to_text_node = SpeechToTextNode(5)

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        speech_to_text_node.run()
        rate.sleep()
