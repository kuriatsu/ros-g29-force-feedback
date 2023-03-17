#!/usr/bin/python3
# -*- coding:utf-8 -*-

import glob
import os
import sys
try:
    sys.path.append(glob.glob('**/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
    sys.path.append("./")
except IndexError:
    pass

# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================
import carla
import numpy as np
import random
import rospy
from ros_g29_force_feedback.msg import ForceFeedback

class CarlaFFNode():
    def __init__(self):
        client = carla.Client("127.0.0.1", 2000)
        client.set_timeout(2.0)
        self.world = client.get_world()
        self.actor = self.get_hero()

        self.publisher = rospy.Publisher("/ff_target", ForceFeedback, queue_size=10)
        rospy.Timer(rospy.Duration(0.1), self.timer_cb)

    def get_hero(self):
        for actor in self.world.get_actors():
            if actor.attributes.get("role_name") in ["hero", "ego_vehicle"]:
                return actor
        raise RuntimeError("Could not find hero or ego_vehicle actor in the simulation.")

            
    def timer_cb(self,event=None):
        out_msg = ForceFeedback()
        out_msg.header.stamp = rospy.Time.now()
        
        steering_angle = self.actor.get_control().steer
        out_msg.position = steering_angle
        out_msg.torque = 0.8
        self.publisher.publish(out_msg)

def main(args=None):

    rospy.init_node("carla_ff_node")
    carla_ff_node = CarlaFFNode()
    rospy.spin()


if __name__ == "__main__":
    main()
