#! /usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright (c) 2018 TOYOTA MOTOR CORPORATION
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

#  * Redistributions of source code must retain the above copyright notice,
#  this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright
#  notice, this list of conditions and the following disclaimer in the
#  documentation and/or other materials provided with the distribution.
#  * Neither the name of Toyota Motor Corporation nor the names of its
#  contributors may be used to endorse or promote products derived from
#  this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
import os
import random
import unittest

from geometry_msgs.msg import Twist
import numpy as np
from PIL import Image
import rosnode
import rospkg
import rospy
import rostest
import tf

# map上のfree画素値
_FREE = 254


def count_map_free(pgm_file):
    u"""pgmのfreeな画素の数を返す"""
    im = np.asarray(Image.open(pgm_file))
    return len(np.where(im == _FREE)[0])


class TestMapping(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rospy.init_node('test_hsrb_mapping')

    def setUp(self):
        self.pub_cmd_vel = rospy.Publisher(
            '/hsrb/command_velocity', Twist, queue_size=10)
        self.listener = tf.TransformListener()
        rospy.sleep(5.0)

    def test_running(self):
        nodes = rosnode.get_node_names()
        self.assertIn('/gazebo', nodes, 'node does not exit')
        self.assertIn('/gmapping', nodes, 'node does not exit')

    def test_map(self):
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.5
        cmd_vel.angular.z = 1.0

        for i in range(200):
            self.pub_cmd_vel.publish(cmd_vel)
            rospy.sleep(0.1)
        # Save map image
        id_num = random.randint(1000, 10000)
        map_result = '/tmp/map_{}'.format(id_num)
        os.system("rosrun map_server map_saver -f {}".format(map_result))

        rospack = rospkg.RosPack()
        map_answer = rospack.get_path(
            'hsrb_mapping') + '/test/map_answer.pgm'
        rospy.loginfo("map_answer: {}".format(map_answer))

        # Check if the map is different
        answer = count_map_free(map_answer)
        result = count_map_free(map_result + '.pgm')
        self.assertAlmostEqual(answer, result, delta=100)


if __name__ == '__main__':
    rostest.rosrun('hsrb_mapping', 'test_hsrb_mapping', TestMapping)
