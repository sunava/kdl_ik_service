#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import moveit_msgs.srv
import geometry_msgs.msg
import kdl_ik_service.ik
import signal


class IkService(Node):

    def __init__(self):
        super().__init__('kdl_ik_service')
        self.srv = self.create_service(moveit_msgs.srv.GetPositionIK, 'get_ik', self.callback)
        self.get_logger().info('IK server ready.')
        self.declare_parameter('ik_debug', False)
        self.declare_parameter('robot_description', None)

    def callback(self, request, response):
        # print "Got request %s"%(request)
        self.get_logger().debug("Got request %s" % (request))
        # debug_mode = rospy.get_param("ik_debug", False)
        debug_mode = self.get_parameter("ik_debug")
        ik_logger = self.get_logger()

        # response = moveit_msgs.srv.GetPositionIKResponse()

        end_effector_link = request.ik_request.ik_link_name
        ik_logger.debug(f"End Effector Link: {end_effector_link}")

        pose_stamped = request.ik_request.pose_stamped
        transform_stamped = geometry_msgs.msg.TransformStamped
        transform_stamped.header = pose_stamped.header
        transform_stamped.child_frame_id = end_effector_link
        transform_stamped.transform = geometry_msgs.msg.Transform
        transform_stamped.transform.translation = geometry_msgs.msg.Vector3
        transform_stamped.transform.translation.x = pose_stamped.pose.position.x
        transform_stamped.transform.translation.y = pose_stamped.pose.position.y
        transform_stamped.transform.translation.z = pose_stamped.pose.position.z
        transform_stamped.transform.rotation = pose_stamped.pose.orientation
        ik_logger.debug(f"Target pose: {transform_stamped}")

        base_link = transform_stamped.header.frame_id
        ik_logger.debug(f"Base Link: {base_link}")

        joint_state = request.ik_request.robot_state.joint_state
        ik_logger.debug(f"Joint states: {joint_state}")

        timeout = request.ik_request.timeout
        ik_logger.debug(f"Timeout: {timeout}")

        response.solution.joint_state = joint_state
        ik_logger.info("Using NR_JL solver")
        urdf_string = self.get_parameter("robot_description").get_parameter_value().string_value
        new_joint_state_vector, success = kdl_ik_service.ik.calculate_ik(base_link, end_effector_link,
                                                                         joint_state.position, transform_stamped,
                                                                         ik_logger.info, urdf_string, "NR_JL")
        if not success:
            ik_logger.info("NR_JL solver failed. Using LMA solver")
            new_joint_state_vector, success = kdl_ik_service.ik.calculate_ik(base_link, end_effector_link,
                                                                             joint_state.position, transform_stamped,
                                                                             ik_logger.info, urdf_string, "LMA")

        ik_logger.debug(f"Calculates Joint states{new_joint_state_vector}")
        response.solution.joint_state.position = new_joint_state_vector

        if success:
            response.error_code.val = response.error_code.SUCCESS
        else:
            response.error_code.val = response.error_code.NO_IK_SOLUTION
        ik_logger.debug(f"Response: {response}")
        return response


def main():
    signal.signal(signal.SIGINT, signal_handler)

    rclpy.init()
    ik_service = IkService()
    rclpy.spin(ik_service)
    rclpy.shutdown()

def signal_handler(sig, frame):
    exit(0)

class Logger(object):
    def __init__(self, display_output=False):
        self.display_output = display_output

    def log(self, line):
        if self.display_output:
            print(line)


if __name__ == "__main__":
    main()
