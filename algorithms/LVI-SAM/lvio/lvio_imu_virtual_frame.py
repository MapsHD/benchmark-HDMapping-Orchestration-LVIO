#!/usr/bin/env python
"""Re-express an IMU in a virtual frame V that has the same origin but rotated axes.

LVI-SAM hard-codes its own rig's LiDAR <-> IMU axes (R_lidar_imu = Ry(180 deg)). On COCO the
Alphasense IMU is ~Rx(180 deg) from the LiDAR, so this node rotates every sample into V with
R_lidar_V = Ry(180 deg) exactly:  omega_V = R * omega,  acc_V = R * acc  (R = ~rotation, row-major).
Orientation is not passed through (the input has none); imu_filter_madgwick adds it downstream.
See algorithms/LVI-SAM/NOTES.md. Python 2 (Melodic) and 3 compatible.

Private params: ~rotation (9 floats, row-major R_V_imu), ~frame_id (output frame id),
~acc_scale (factor applied to the acceleration, default 1; 9.80665 for an IMU reporting g, e.g. Livox).
Topics: imu_in (sensor_msgs/Imu) -> imu_out (sensor_msgs/Imu); remap them in the launch file.
"""
import rospy
from sensor_msgs.msg import Imu


def rotate(r, v):
    return (r[0] * v.x + r[1] * v.y + r[2] * v.z,
            r[3] * v.x + r[4] * v.y + r[5] * v.z,
            r[6] * v.x + r[7] * v.y + r[8] * v.z)


def main():
    rospy.init_node('lvio_imu_virtual_frame')
    r = [float(x) for x in rospy.get_param('~rotation')]
    if len(r) != 9:
        raise ValueError('~rotation must have 9 values')
    frame_id = rospy.get_param('~frame_id', 'lvio_imu_v')
    acc_scale = float(rospy.get_param('~acc_scale', 1.0))
    pub = rospy.Publisher('imu_out', Imu, queue_size=2000)

    def callback(msg):
        out = Imu()
        out.header = msg.header
        out.header.frame_id = frame_id
        out.orientation_covariance[0] = -1.0  # no orientation
        out.angular_velocity.x, out.angular_velocity.y, out.angular_velocity.z = rotate(r, msg.angular_velocity)
        out.linear_acceleration.x, out.linear_acceleration.y, out.linear_acceleration.z = \
            [acc_scale * a for a in rotate(r, msg.linear_acceleration)]
        out.angular_velocity_covariance = msg.angular_velocity_covariance
        out.linear_acceleration_covariance = msg.linear_acceleration_covariance
        pub.publish(out)

    rospy.Subscriber('imu_in', Imu, callback, queue_size=2000, tcp_nodelay=True)
    rospy.spin()


if __name__ == '__main__':
    main()
