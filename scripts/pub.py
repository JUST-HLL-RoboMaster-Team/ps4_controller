#!/home/wangzihanggg/miniconda3/envs/pytorch/bin/python
# -*-. coding: utf-8 -*-

import rospy
from std_msgs.msg import Bool
from pyPS4Controller.controller import Controller

def connect():
    print("ps4 controller is connected")


def disconnect():
    print("ps4 controller was disconnected")

class MyController(Controller):

    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)
        self.pub = rospy.Publisher("ps4_message", Bool, queue_size=10)
        self.msg = Bool()
        self.r1_state = False 

    def connect(self):
        print("\033[32m" + "ps4 controller is connected" + "\033[0m")


    def disconnect(self):
        print("\033[32m" + "ps4 controller is disconnected" + "\033[0m")


    def on_R1_press(self):
        self.r1_state = not self.r1_state  # Toggle the state
        self.publish_msg()

    def on_R1_release(self):
        pass  # You can leave this empty if no action is needed on release

    def publish_msg(self):
        self.msg.data = self.r1_state
        self.pub.publish(self.msg)
        rospy.loginfo("Published data: %s", self.msg.data)

if __name__ == "__main__":
    rospy.init_node("ps4_controller")
    controller = MyController(interface="/dev/input/js0", connecting_using_ds4drv=False)
    controller.listen()

