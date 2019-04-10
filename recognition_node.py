#/usr/bin/python3
import rclpy
from roboy_cognition_msgs.msg import RecognizedSpeech
import speech_recognition as sr
import threading
from ros2_speech_recognition.recognizer import *


BING_KEY = "e91e7a4512aa48b3b53b36b56bd1feb7"

def listener(source, node):
    publisher = node.create_publisher(RecognizedSpeech,\
                                '/roboy/cognition/speech/recognition')

    bing = BingVoice(BING_KEY)
    msg = RecognizedSpeech()
    try:
        msg.source = source.id
    except:
        msg.source = -1

    # with source as source:
    while True:
        input("Press enter to listen again...")
        #     audio = recognizer.listen(source=source)
        try:
            text = bing.stt_with_vad(source)
            msg.text = text
            publisher.publish(msg)
            print("source %i: "%msg.source + text)
        except sr.UnknownValueError:
            print("source %i: Microsoft Bing Voice Recognition could not understand audio"%msg.source)
        except sr.RequestError as e:
            print("{1}: Could not request results from Microsoft Bing Voice Recognition service; {0}".format(e, msg.source))


def odas_recognition(node):
    from ros2_speech_recognition.odas_sr_driver import Odas
    print("waiting for odas connection")
    o = Odas(port=10002, chunk_size=2048)

    listeners = []
    for i in range(4):
        listeners.append(threading.Thread(target=listener, \
                                        args = (o.channels[i], node, )))
    for l in listeners:
        l.start()


def mic_recognition(node):
    mic = sr.Microphone()
    listener(mic, node)

def main(args=None):
    rclpy.init()
    node = rclpy.create_node('odas_speech_recognition')
    mic_recognition(node)
    # odas_recognition(r, node) # requires RPi running odas

if __name__ == '__main__':
    main()
