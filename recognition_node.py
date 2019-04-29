#/usr/bin/python3
import rclpy
from roboy_cognition_msgs.msg import RecognizedSpeech
from roboy_cognition_msgs.srv import RecognizeSpeech
import speech_recognition as sr
import threading
from recognizer import *
from time import sleep

BING_KEY = ""

def callback(request, response):
    global bing, src
    # msg = RecognizedSpeech()
    # try:
    #     msg.source = source.id
    # except:
    #     msg.source = -1
    try:
        text = bing.stt_with_vad(src)
        response.text = text
        # msg.text = text
        # publisher.publish(msg)
        print("text: " + text)
    except sr.UnknownValueError:
        print("Microsoft Bing Voice Recognition could not understand audio")
    except sr.RequestError as e:
        print("Could not request results from Microsoft Bing Voice Recognition service; {0}".format(e,))

    return response

def listener(source, node):
    publisher = node.create_publisher(RecognizedSpeech,\
                                '/roboy/cognition/speech/recognition')
    global bing, src
    src = source
    bing = BingVoice(BING_KEY)
    srv = node.create_service(RecognizeSpeech, \
        '/roboy/cognition/speech/recognition', callback)

    # with source as source:
    while rclpy.ok():
        rclpy.spin_once(node)

    node.destroy_service(srv)
    node.destroy_publisher(publisher)
    rclpy.shutdown()


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

def odas_single_channel(node):
    from ros2_speech_recognition.odas_sr_driver import Odas
    print("waiting for odas connection")
    o = Odas(port=10002, chunk_size=2048)
    listener(o.channels[0], node)

def mic_recognition(node):
    mic = sr.Microphone()
    listener(mic, node)

def main(args=None):
    rclpy.init()
    node = rclpy.create_node('odas_speech_recognition')
    mic_recognition(node)
    # odas_single_channel(node) # requires RPi running odas
    # odas_recognition(node)

if __name__ == '__main__':
    main()
