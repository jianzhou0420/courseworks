#!/usr/bin/env python3

import rospy
import numpy as np

# TODO: Include all the required service classes
# your code starts here -----------------------------
from cw1q4_srv.srv import quat2rodrigues, quat2zyx, quat2rodriguesResponse, quat2zyxResponse, quat2rodriguesRequest, \
    quat2zyxRequest


# your code ends here -------------------------------


def convert_quat2zyx(request):
    # TODO complete the function
    """Callback ROS service function to convert quaternion to Euler z-y-x representation

    Args:
        request (quat2zyxRequest): cw1q4_srv service message, containing
        the quaternion you need to convert.

    Returns:
        quat2zyxResponse: cw1q4_srv service response, in which 
        you store the requested euler angles 
    """
    assert isinstance(request, quat2zyxRequest)

    # Your code starts here ----------------------------
    response = quat2zyxResponse()

    qx, qy, qz, qw = request.q.x, request.q.y, request.q.z, request.q.w

    response.z = np.arctan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx ** 2 + qy ** 2))
    response.y = np.arcsin(2 * (qw * qy - qz * qx))
    response.x = np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy ** 2 + qz ** 2))

    # Your code ends here ------------------------------

    assert isinstance(response, quat2zyxResponse)
    return response


def convert_quat2rodrigues(request):
    # TODO complete the function

    """Callback ROS service function to convert quaternion to rodrigues representation
    
    Args:
        request (quat2rodriguesRequest): cw1q4_srv service message, containing
        the quaternion you need to convert

    Returns:
        quat2rodriguesResponse: cw1q4_srv service response, in which 
        you store the requested rodrigues representation 
    """
    assert isinstance(request, quat2rodriguesRequest)

    # Your code starts here ----------------------------
    qx, qy, qz, qw = request.q.x, request.q.y, request.q.z, request.q.w
    response = quat2rodriguesResponse()

    R = np.zeros((3, 3))

    R[0][0] = 1 - 2 * qy ** 2 - 2 * qz ** 2
    R[0][1] = 2 * qx * qy - 2 * qz * qw
    R[0][2] = 2 * qx * qz + 2 * qy * qw
    R[1][0] = 2 * qx * qy + 2 * qz * qw
    R[1][1] = 1 - 2 * qx ** 2 - 2 * qz ** 2
    R[1][2] = 2 * qy * qz - 2 * qx * qw
    R[2][0] = 2 * qx * qz - 2 * qy * qw
    R[2][1] = 2 * qy * qz + 2 * qx * qw
    R[2][2] = 1 - 2 * qx ** 2 - 2 * qy ** 2
    theta = np.arccos((R[0][0] + R[1][1] + R[2][2] - 1) / 2)

    response.x = theta * (R[2][1] - R[1][2]) / 2 / np.sin(theta)
    response.y = theta * (R[0][2] - R[2][0]) / 2 / np.sin(theta)
    response.z = theta * (R[1][0] - R[0][1]) / 2 / np.sin(theta)

    # Your code ends here ------------------------------

    assert isinstance(response, quat2rodriguesResponse)
    return response


def rotation_converter():
    rospy.init_node('rotation_converter')

    # Initialise the services
    rospy.Service('quat2rodrigues', quat2rodrigues, convert_quat2rodrigues)
    rospy.Service('quat2zyx', quat2zyx, convert_quat2zyx)

    rospy.spin()


if __name__ == "__main__":
    rotation_converter()
