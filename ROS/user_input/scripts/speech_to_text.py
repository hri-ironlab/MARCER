#!/usr/bin/env python3
import torch
import gc
import numpy as np
import atexit
import whisper
import time
import pyaudio
import string
import rospy
from std_msgs.msg import String
from alive_progress import alive_bar
import time, os, sys, contextlib
from std_msgs.msg import Float32
# Suppress errors looking for device index
@contextlib.contextmanager
def ignoreStderr():
    devnull = os.open(os.devnull, os.O_WRONLY)
    old_stderr = os.dup(2)
    sys.stderr.flush()
    os.dup2(devnull, 2)
    os.close(devnull)
    try:
        yield
    finally:
        os.dup2(old_stderr, 2)
        os.close(old_stderr)

class SpeechToTextNode:
    def __init__(self, chunk, rate, channels, format, device_index, record_time):
        # Initialize the ROS node
        rospy.init_node("speech_to_text_node")

        self.progress_publisher = rospy.Publisher('/user_input/listening_progress', Float32, queue_size=10)

        self.key_phrase = "hey fetch"
        self.model = whisper.load_model("small")
        with ignoreStderr():
            self.audio_interface = pyaudio.PyAudio().open(format=format,
                            channels=channels,
                            rate=rate,
                            input=True,
                            input_device_index=device_index,
                            frames_per_buffer=chunk)

        self.speech_publisher = rospy.Publisher('user_input/speech', String, queue_size=10)
        self.chunk = chunk
        self.record_time = record_time
        self.progress = Float32()

    def exit_handler(self):
        del self.model
        torch.cuda.empty_cache()
        gc.collect()

    # Process and transcribe audio data
    def process_audio(self, audio_data, language="en"):
        audio = np.frombuffer(audio_data, dtype=np.int16).astype(np.float32)*(1/32768.0)
        audio = audio.astype(np.float32)
        audio = whisper.pad_or_trim(audio)
        mel = whisper.log_mel_spectrogram(audio).to(self.model.device)

        options = whisper.DecodingOptions(fp16 = False,  language=language)
        result = whisper.decode(self.model, mel, options)

        return result.text
    
    def run(self):
        # Main functionality for real-time audio transcription
        audio_data = b""

        try:
            print('Listening for "Hey Fetch [COMMAND]"...')
            with alive_bar(self.record_time, title='', spinner=None, monitor=False, stats=False, theme='classic') as progress_bar:
                for i in range(self.record_time):
                    self.progress.data = i / self.record_time
                    self.progress_publisher.publish(self.progress)
                    data = self.audio_interface.read(self.chunk, exception_on_overflow=False)
                    audio_data += data
                    time.sleep(.005)
                    progress_bar()
        except KeyboardInterrupt:
            print("\nRecording interrupted by user.")

        # Process and transcribe the audio
        transcript = self.process_audio(audio_data)
        print("Transcript:", transcript)
        transcript = transcript.translate(str.maketrans('', '', string.punctuation)).lower()
        if self.key_phrase in transcript:
            task = transcript.split(self.key_phrase)[1]
            self.speech_publisher.publish(task)
            print("\nSending command:", task)
        
        print("\n")

if __name__ == "__main__":
    speech_to_text_node = SpeechToTextNode(1024, 16000, 1, pyaudio.paInt16, None, 200)
    atexit.register(speech_to_text_node.exit_handler)

    rospy.sleep(3)    
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        speech_to_text_node.run()
        rate.sleep()
