#!/usr/bin/env python

def node():
    """
    Node to detect the container entry in a laser scan.
    """
    global TF_BUFFER
    rospy.init_node("detect_container")
    TF_BUFFER = tf2_ros.Buffer()
    tf2_ros.TransformListener(TF_BUFFER)
    server = DetectionServer()
    rospy.spin()


if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass