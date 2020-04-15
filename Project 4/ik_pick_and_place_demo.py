#!/usr/bin/env python

# Copyright (c) 2013-2015, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
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

"""
Baxter RSDK Inverse Kinematics Pick and Place Demo
"""
import argparse
import struct
import sys
import copy
import time
import rospy
import rospkg

from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import (
    Header,
    Empty,
)

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

import baxter_interface

class PickAndPlace(object):
    def __init__(self, limb, hover_distance = 0.15, verbose=True):
        self._limb_name = limb # string
        self._hover_distance = hover_distance # in meters
        self._verbose = verbose # bool
        self._limb = baxter_interface.Limb(limb)
        self._gripper = baxter_interface.Gripper(limb)
        ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        self._iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        rospy.wait_for_service(ns, 5.0)
        # verify robot is enabled
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()

    def move_to_start(self, start_angles=None):
        print("Moving the {0} arm to start pose...".format(self._limb_name))
        if not start_angles:
            start_angles = dict(zip(self._joint_names, [0]*7))
        self._guarded_move_to_joint_position(start_angles)
        self.gripper_open()
        rospy.sleep(1.0)
        print("Running. Ctrl-c to quit")

    def ik_request(self, pose):
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        ikreq = SolvePositionIKRequest()
        ikreq.pose_stamp.append(PoseStamped(header=hdr, pose=pose))
        try:
            resp = self._iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return False
        # Check if result valid, and type of seed ultimately used to get solution
        # convert rospy's string representation of uint8[]'s to int's
        resp_seeds = struct.unpack('<%dB' % len(resp.result_type), resp.result_type)
        limb_joints = {}
        if (resp_seeds[0] != resp.RESULT_INVALID):
            seed_str = {
                        ikreq.SEED_USER: 'User Provided Seed',
                        ikreq.SEED_CURRENT: 'Current Joint Angles',
                        ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                       }.get(resp_seeds[0], 'None')
            if self._verbose:
                print("IK Solution SUCCESS - Valid Joint Solution Found from Seed Type: {0}".format(
                         (seed_str)))
            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            if self._verbose:
                print("IK Joint Solution:\n{0}".format(limb_joints))
                print("------------------")
        else:
            rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
            return False
        return limb_joints

    def _guarded_move_to_joint_position(self, joint_angles):
        if joint_angles:
            self._limb.move_to_joint_positions(joint_angles)
        else:
            rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")

    def gripper_open(self):
        self._gripper.open()
        rospy.sleep(1.0)

    def gripper_close(self):
        self._gripper.close()
        rospy.sleep(1.0)

    def _approach(self, pose):
        approach = copy.deepcopy(pose)
        # approach with a pose the hover-distance above the requested pose
        approach.position.z = approach.position.z + self._hover_distance
        joint_angles = self.ik_request(approach)
        self._guarded_move_to_joint_position(joint_angles)

    def _retract(self):
        # retrieve current pose from endpoint
        current_pose = self._limb.endpoint_pose()
        ik_pose = Pose()
        ik_pose.position.x = current_pose['position'].x
        ik_pose.position.y = current_pose['position'].y
        ik_pose.position.z = current_pose['position'].z + self._hover_distance
        ik_pose.orientation.x = current_pose['orientation'].x
        ik_pose.orientation.y = current_pose['orientation'].y
        ik_pose.orientation.z = current_pose['orientation'].z
        ik_pose.orientation.w = current_pose['orientation'].w
        joint_angles = self.ik_request(ik_pose)
        # servo up from current pose
        self._guarded_move_to_joint_position(joint_angles)

    def _servo_to_pose(self, pose):
        # servo down to release
        joint_angles = self.ik_request(pose)
        self._guarded_move_to_joint_position(joint_angles)

    def pick(self, pose):
        # open the gripper
        self.gripper_open()
        # servo above pose
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        # close gripper
        self.gripper_close()
        # retract to clear object
        self._retract()

    def place(self, pose):
        # servo above pose
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        # open the gripper
        self.gripper_open()
        # retract to clear object
        self._retract()
    def simply_move(self, pose):
        # servo above pose
        self.gripper_close()

        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        # retract to clear object
        # self._retract()
        # self.gripper_close()


def load_gazebo_models(table_pose=Pose(position=Point(x=1.05, y=0.0, z=0.0)),
                       table_reference_frame="world",
                       block_pose=Pose(position=Point(x=0.75, y=0.0265, z=0.7825)),
                       block_reference_frame="world", table_pose1 = Pose(position=Point(x=0.24, y= 1.19, z = 0.0)), table_reference_frame1 = "world",
                       block_pose1 = Pose(position=Point(x= 0.624727, y=1.181466, z=0.85)), block_reference_frame1="world", block_pose2 = Pose(position=Point(x= 1.063386, y=0.368854, z=0.85)), block_reference_frame2="world", block_pose3 = Pose(position=Point(x= 0.3, y=0.8, z=0.85)), block_reference_frame3="world") :
    # Get Models' Path
    model_path = rospkg.RosPack().get_path('baxter_sim_examples')+"/models/"
    # Load Table SDF
    table_xml = ''
    with open (model_path + "cafe_table/model.sdf", "r") as table_file:
        table_xml=table_file.read().replace('\n', '')

    
    # Load Block URDF
    block_xml = ''
    with open (model_path + "block/model.urdf", "r") as block_file:
        block_xml=block_file.read().replace('\n', '')

    # Load obstacle Block URDF
    block_xml1 = ''
    with open (model_path + "block/model1.urdf", "r") as block_file:
        block_xml1=block_file.read().replace('\n', '')

    # Load obstacle Block 2 URDF
    block_xml2 = ''
    with open (model_path + "block/model2.urdf", "r") as block_file:
        block_xml2=block_file.read().replace('\n', '')

    block_xml3 = ''
    with open (model_path + "block/model3.urdf", "r") as block_file:
        block_xml3=block_file.read().replace('\n', '')

    # Spawn Table SDF
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf("cafe_table", table_xml, "/",
                             table_pose, table_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))
    # Spawn Block URDF
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf = spawn_urdf("block", block_xml, "/",
                               block_pose, block_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn URDF service call failed: {0}".format(e))

    # Spawn side table
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf("cafe_table1", table_xml, "/",
                             table_pose1, table_reference_frame1)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))

    # Spawn Obstacle Block URDF
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf = spawn_urdf("block1", block_xml1, "/",
                               block_pose1, block_reference_frame1)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn URDF service call failed: {0}".format(e))
  
    # Spawn Obstacle Block 2 URDF
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf = spawn_urdf("block2", block_xml2, "/",
                               block_pose2, block_reference_frame2)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn URDF service call failed: {0}".format(e))

    # Spawn Obstacle Block 3 URDF
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf = spawn_urdf("block3", block_xml3, "/",
                               block_pose3, block_reference_frame3)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn URDF service call failed: {0}".format(e))


def delete_gazebo_models():
    # This will be called on ROS Exit, deleting Gazebo models
    # Do not wait for the Gazebo Delete Model service, since
    # Gazebo should already be running. If the service is not
    # available since Gazebo has been killed, it is fine to error out
    try:
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        resp_delete = delete_model("cafe_table")
        resp_delete = delete_model("cafe_table1")
        resp_delete = delete_model("block")
        resp_delete = delete_model("block1")
        resp_delete = delete_model("block2")
        resp_delete = delete_model("block3")
    except rospy.ServiceException, e:
        rospy.loginfo("Delete Model service call failed: {0}".format(e))

def main():
    """RSDK Inverse Kinematics Pick and Place Example

    A Pick and Place example using the Rethink Inverse Kinematics
    Service which returns the joint angles a requested Cartesian Pose.
    This ROS Service client is used to request both pick and place
    poses in the /base frame of the robot.

    Note: This is a highly scripted and tuned demo. The object location
    is "known" and movement is done completely open loop. It is expected
    behavior that Baxter will eventually mis-pick or drop the block. You
    can improve on this demo by adding perception and feedback to close
    the loop.
    """
    rospy.init_node("ik_pick_and_place_demo")
    # Load Gazebo Models via Spawning Services
    # Note that the models reference is the /world frame
    # and the IK operates with respect to the /base frame
    load_gazebo_models()
    # Remove models from the scene on shutdown
    rospy.on_shutdown(delete_gazebo_models)

    # Wait for the All Clear from emulator startup
    rospy.wait_for_message("/robot/sim/started", Empty)

    limb = 'left'
    hover_distance = 0.15 # meters
    # Starting Joint angles for left arm
    starting_joint_angles = {'left_w0': 0.6699952259595108,
                             'left_w1': 1.030009435085784,
                             'left_w2': -0.4999997247485215,
                             'left_e0': -1.189968899785275,
                             'left_e1': 1.9400238130755056,
                             'left_s0': -0.08000397926829805,
                             'left_s1': -0.9999781166910306}
    pnp = PickAndPlace(limb, hover_distance)
    # An orientation for gripper fingers to be overhead and parallel to the obj
    overhead_orientation = Quaternion(
                             x=-0.0249590815779,
                             y=0.999649402929,
                             z=0.00737916180073,
                             w=0.00486450832011)
    block_poses = list()

#first block is here
    block_poses.append(Pose(
        position=Point(x=0.77, y=0.0510, z=-0.170), #0.0265 changed to 0.0165
        orientation=overhead_orientation))
#first stop
    block_poses.append(Pose(
        position=Point(x=0.611, y=0.102, z=0.2), #0.0265 changed to 0.0165
        orientation=overhead_orientation))
#second stop
    block_poses.append(Pose(
        position=Point(x=0.412, y=0.309, z=0.2), #0.0265 changed to 0.0165
        orientation=overhead_orientation))
#placing
    block_poses.append(Pose(
        position=Point(x=0.1898577, y=0.8511811, z=-0.170), #0.0265 changed to 0.0165
        orientation=overhead_orientation))

# #picking the second block is here
    block_poses.append(Pose(
        position=Point(x=0.332, y=0.821, z=-0.170),
        orientation=overhead_orientation))
    # Feel free to add additional desired poses for the object.
    # Each additional pose will get its own pick and place.
    # Move to the desired starting angles
    pnp.move_to_start(starting_joint_angles)
    idx = 0
    while not rospy.is_shutdown():
#PICK UP       
        pnp.pick(block_poses[idx])
        #pick up the first block
        print '----------------'
        print 'Red cube picked!'
        print '----------------'
#point1
        idx = (1)# % len(block_poses)
        pnp.simply_move(block_poses[idx])
#point2
        idx = (2) #% len(block_poses)
        pnp.simply_move(block_poses[idx])
# PLACE DOWN
        idx = (3)# % len(block_poses)
        pnp.place(block_poses[idx])
        print '----------------'
        print 'Red cube placed!'
        print '----------------'
#pick up
        idx = (4)# % len(block_poses)
        pnp.pick(block_poses[idx])
        print '----------------'
        print 'White cube picked!'
        print '----------------'
#point 2
        idx = (2)# % len(block_poses)
        pnp.simply_move(block_poses[idx])
#point 1
        idx = (1) #% len(block_poses)
        pnp.simply_move(block_poses[idx])
#places down
        idx = (0)# % len(block_poses)
        pnp.place(block_poses[idx])
        print '----------------'
        print 'White cube placed!'
        print '----------------'
#point1
        idx = (1) #% len(block_poses)
        pnp.simply_move(block_poses[idx])
        print '----------------'
        print 'Task Complete!'
        print '----------------'
        break
    return 0

if __name__ == '__main__':
    sys.exit(main())

#pick up the first block

#go to 
#{'left_w0': -1.9610191053612303, 'left_w1': -0.39130158977169593, 'left_w2': 2.233642471603942, 'left_e0': -0.5915234984895469, 'left_e1': 1.6155294835628007, 'left_s0': 0.4206389332292275, 'left_s1': -0.7271358350918121}

#go to
#{'left_w0': -2.0501214394773193, 'left_w1': -0.355378824043284, 'left_w2': 2.286137969880706, 'left_e0': -0.5265071811964575, 'left_e1': 1.6171503619459462, 'left_s0': 0.37523343822834004, 'left_s1': -0.7519231383283744}

#go to
#{'left_w0': -1.7779841765120739, 'left_w1': -0.5095807589521648, 'left_w2': 2.174739647310927, 'left_e0': -0.7922760665702202, 'left_e1': 1.5949682414274124, 'left_s0': 0.544932544639182, 'left_s1': -0.6298376416174745}

#DROP

#pick up the next one

#go to
#{'left_w0': -1.3280297509854482, 'left_w1': -0.9107254288125348, 'left_w2': 2.190623837332259, 'left_e0': -1.2864145806663185, 'left_e1': 1.5416502454661867, 'left_s0': 0.7510491181869309, 'left_s1': -0.21537259002941944}


#go to
#{'left_w0': -1.391081280924858, 'left_w1': -0.8391476826900638, 'left_w2': 2.173405192842787, 'left_e0': -1.2092214253651243, 'left_e1': 1.5420723771455063, 'left_s0': 0.7278928494329576, 'left_s1': -0.2860480888367943}

#DROP OFF AT 
#{'left_w0': -1.5516028675133178, 'left_w1': -0.6892121573249039, 'left_w2': 2.1489083243993328, 'left_e0': -1.033530554487903, 'left_e1': 1.5618165114653615, 'left_s0': 0.6637570199835905, 'left_s1': -0.4523764050613869}



