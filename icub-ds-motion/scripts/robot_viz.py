#!/usr/bin/env python
from geometry_msgs.msg import Pose2D, Pose, PoseStamped
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker
import math
import rospy



def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

class ObjectViz(object):
    
    def __init__(self, marker_pub, robot_pub):
        self._object_pose    = []
        self._marker_pub     = marker_pub    
        self._robot_pub      = robot_pub
        rospy.on_shutdown(self.shutdown)


    def callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y 
        z = msg.pose.position.z 
        q_x = msg.pose.orientation.x
        q_y = msg.pose.orientation.y
        q_z = msg.pose.orientation.z
        q_w = msg.pose.orientation.w

        self._robot_pose = [x,y,z,q_x,q_y,q_z,q_w];
        self.viz_cube()
        self.pub_pose()

    def viz_cube(self):
       marker = Marker()
       marker.header.frame_id = "/gazebo_world"
       marker.type = marker.CUBE
       marker.action = marker.ADD
       marker.scale.x = 0.15
       marker.scale.y = 0.30
       marker.scale.z = 1.0
       marker.color.a = 1.0
       marker.color.r = 0.5
       marker.color.g = 0.5
       marker.color.b = 0.5       
       marker.pose.position.x = self._robot_pose[0]
       marker.pose.position.y = self._robot_pose[1]
       marker.pose.position.z = self._robot_pose[2]
       marker.pose.orientation.x = self._robot_pose[3]
       marker.pose.orientation.y = self._robot_pose[4]
       marker.pose.orientation.z = self._robot_pose[5]
       marker.pose.orientation.w = self._robot_pose[6]
       self._marker_pub.publish(marker)
    
    def pub_pose(self):
      robot = Pose()
      robot.position.x = self._robot_pose[0]
      robot.position.y = self._robot_pose[1]
      robot.position.z = self._robot_pose[2]
      robot.orientation.x = self._robot_pose[3]
      robot.orientation.y = self._robot_pose[4]
      robot.orientation.z = self._robot_pose[5]
      robot.orientation.w = self._robot_pose[6]
      self._robot_pub.publish(robot)

    def shutdown(self):
        """
        command executed after Ctrl+C is pressed
        """
        rospy.loginfo("Stop Robot Viz Node")
        rospy.sleep(1)

def main():
    rospy.init_node('robot_viz')
    wait_for_time()
    marker_publisher = rospy.Publisher('object_marker', Marker, queue_size=5)
    robot_publisher  = rospy.Publisher('iCub_pose', Pose, queue_size=5)
    
    objectViz = ObjectViz(marker_publisher, robot_publisher)
    rospy.Subscriber('/icubSim_CoM_pose', PoseStamped, objectViz.callback)
    rospy.spin()

    rospy.loginfo('Running until shutdown (Ctrl-C).')
    while not rospy.is_shutdown():
       objectViz.shutdown()
       rospy.sleep(0.5)

    rospy.loginfo('Node finished')


if __name__ == '__main__':
    main()