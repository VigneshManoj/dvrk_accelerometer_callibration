# !/usr/bin/env python

import rospy
import tf
import geometry_msgs.msg
import numpy as np

from sympy import symbols
from numpy.linalg import inv
from PyKDL import Rotation, Vector, Frame


def diff_calc_func(start_mat, end_mat, corrected_avg_interval):

    temp = start_mat.M.Inverse() * end_mat.M
    ang, temp_axis = Rotation.GetRotAngle(temp)

    o = start_mat.M * temp_axis
    p_start_vec = start_mat.p
    p_end_vec = end_mat.p
    # print "p vectors ", p_end_vec, p_start_vec

    # Finding difference in position values between start and end
    delta_x = p_end_vec[0] - p_start_vec[0]
    delta_y = p_end_vec[1] - p_start_vec[1]
    delta_z = p_end_vec[2] - p_start_vec[2]

    # Calculation of differentiated values such as velocity/acceleration
    twist_val = geometry_msgs.msg.Vector3()
    twist_val.x = delta_x / corrected_avg_interval.to_sec()
    twist_val.y = delta_y / corrected_avg_interval.to_sec()
    twist_val.z = delta_z / corrected_avg_interval.to_sec()
    # twist_rot = geometry_msgs.msg.Vector3()

    # Calculation of angular velocity/acceleration
    twist_rot = o * (ang/corrected_avg_interval.to_sec())
    # print "accel is ", twist_vel
    # print "accel rot is ", twist_rot[2]
    return twist_val, twist_rot


if __name__ == '__main__':
    rospy.init_node('daVinci_tf_listener_14')

    listener = tf.TransformListener()
    # print "Reached here"
    # Creation of publishers
    accelerometer_pub = rospy.Publisher('accelerometer_pub', geometry_msgs.msg.WrenchStamped, queue_size=1)
    est_acceleration_pub = rospy.Publisher('est_acceleration_pub', geometry_msgs.msg.WrenchStamped, queue_size=1)
    position_diff_pub = rospy.Publisher('final_position_pub', geometry_msgs.msg.Vector3, queue_size=1)
    x_val_pub = rospy.Publisher('est_x_val', geometry_msgs.msg.Vector3, queue_size=1)

    rate = rospy.Rate(1000.0)
    while not rospy.is_shutdown():
        try:

            # Assigning tracking and reference frame
            tracking_frame = 'psm1_main_insertion_link'
            reference_frame = 'psm1_remote_center_link'

            # Calculation of average time interval, start and end time
            time = rospy.Time(0)
            average_interval = rospy.Duration(0.1)
            # Taking the position, linear and angular velocity of psm1_pitch_back link wrt base_link
            latest_time = listener.getLatestCommonTime(reference_frame, tracking_frame)
            if rospy.Time() == time:
                target_time = latest_time
            else:
                target_time = time
            end_time = min(target_time + average_interval * 0.5, latest_time)
            start_time = max(rospy.Time.from_seconds(0.00001) + average_interval, end_time) - average_interval
            corrected_average_interval = end_time - start_time

            # Retrieve start and end position and quaternion values of tracking frame wrt reference frame
            start_pos, start_quat = listener.lookupTransform(tracking_frame, reference_frame, start_time)
            end_pos, end_quat = listener.lookupTransform(tracking_frame, reference_frame, end_time)

            # Retrieve start and end velocity and angular velocity values of tracking frame wrt reference frame
            (lin_twist_start, ang_twist_start) = listener.lookupTwist(reference_frame,tracking_frame,
                                                                            start_time, rospy.Duration(0.1))
            (lin_twist_end, ang_twist_end) = listener.lookupTwist(reference_frame,tracking_frame,
                                                                            end_time, rospy.Duration(0.1))
            # print "start quat is ", start_quat

            # Create matrices for calculation of velocity which can be given into the custom function
            start_mat_vel = Frame(Rotation.Quaternion(start_quat[0], start_quat[1], start_quat[2], start_quat[3]),
                              Vector(start_pos[0], start_pos[1], start_pos[2]))
            end_mat_vel = Frame(Rotation.Quaternion(end_quat[0], end_quat[1], end_quat[2], end_quat[3]),
                            Vector(end_pos[0], end_pos[1], end_pos[2]))

            # Create matrices for calculation of acceleration which can be given into the custom function
            start_mat_acc = Frame(Rotation.Quaternion(start_quat[0], start_quat[1], start_quat[2], start_quat[3]), Vector(lin_twist_start[0], lin_twist_start[1], lin_twist_start[2]))
            end_mat_acc = Frame(Rotation.Quaternion(end_quat[0], end_quat[1], end_quat[2], end_quat[3]),
                            Vector(lin_twist_end[0], lin_twist_end[1], lin_twist_end[2]))

            # Create matrices for calculation of angular acceleration which can be given into the custom function
            # start_mat_ang_acc = Frame(Rotation.Quaternion(start_quat[0], start_quat[1], start_quat[2], start_quat[3]), Vector(ang_twist_start[0], ang_twist_start[1], ang_twist_start[2]))
            # end_mat_ang_acc = Frame(Rotation.Quaternion(end_quat[0], end_quat[1], end_quat[2], end_quat[3]),
            #                 Vector(ang_twist_end[0], ang_twist_end[1], ang_twist_end[2]))

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        print "----------------------starts here------------------------------------"

        # Calculation of velocity and acceleration values based on start and end position/velocity
        twist_vel, twist_rot_vel = diff_calc_func(start_mat_vel, end_mat_vel, corrected_average_interval)

        twist_acc, randomvar1 = diff_calc_func(start_mat_acc, end_mat_acc, corrected_average_interval)
        # print twist_acc, twist_rot_acc

        # Create matrices based on the omega value at start and end position
        omega_mat_start = np.array([[0, -1 * ang_twist_start[2], ang_twist_start[1]],
                                   [ang_twist_start[2], 0, -1 * ang_twist_start[0]],
                                   [-1 * ang_twist_start[1], ang_twist_start[0], 0]])
        omega_mat_end = np.array(
            [[0, -1 * ang_twist_end[2], ang_twist_end[1]],
             [ang_twist_end[2], 0, -1 * ang_twist_end[0]],
             [-1 * ang_twist_end[1], ang_twist_end[0], 0]])

        # Create variables and assigning required values for ease of readability
        a1, b1, c1, a2, b2, c2 = ang_twist_start[0], ang_twist_start[1], ang_twist_start[2], ang_twist_end[0], ang_twist_end[1], ang_twist_end[2]
        tim_t = corrected_average_interval.to_sec()
        pos_using_vel_x = (twist_vel.x * tim_t)
        pos_using_vel_y = (twist_vel.y * tim_t)
        pos_using_vel_z = (twist_vel.z * tim_t)
        # print "pos using vel ", pos_using_vel_x, pos_using_vel_y, pos_using_vel_z

        # Creation of matrices A and B for solving using formula X=inv(A)*B

        if abs(c1 / tim_t) < 0.01:
            c1_time = 0
        else:
            c1_time = c1 / tim_t
        if abs(c2 / tim_t) < 0.01:
            c2_time = 0
        else:
            c2_time = c2 / tim_t
        if abs(b1 / tim_t) < 0.01:
            b1_time = 0
        else:
            b1_time = b1 / tim_t
        if abs(b2 / tim_t) < 0.01:
            b2_time = 0
        else:
            b2_time = b2 / tim_t
        if abs(a1 / tim_t) < 0.01:
            a1_time = 0
        else:
            a1_time = a1 / tim_t
        if abs(a2 / tim_t) < 0.01:
            a2_time = 0
        else:
            a2_time = a2 / tim_t

        mat_A = [[0, c1_time, -b1_time, 0, -c2_time, b2_time],
                 [-c1_time, 0, a1_time, c2_time, 0, -a2_time],
                 [b1_time, -b2_time, -a1_time, a2_time, 0, 0],
                 [1, 0, 0, -1, 0, 0],
                 [0, 1, 0, 0, -1, 0],
                 [0, 0, 1, 0, 0, -1]]

        mat_B = [twist_acc.x, twist_acc.y, twist_acc.x, -pos_using_vel_x, -pos_using_vel_y, -pos_using_vel_z]
        # print "mat a is ", mat_A
        x_val = np.dot((np.linalg.pinv(mat_A)), mat_B)
        print "Solved value of x is ", x_val[0], x_val[1], x_val[2]

        # To calculate acceleration value based on the estimated position value and check if it is correct
        r2 = np.array([[x_val[0]], [x_val[1]], [x_val[2]]])
        end_pos_it_val_x = x_val[0] + pos_using_vel_x
        end_pos_it_val_y = x_val[1] + pos_using_vel_y
        end_pos_it_val_z = x_val[2] + pos_using_vel_z

        r2_end = np.array([[end_pos_it_val_x], [end_pos_it_val_y], [end_pos_it_val_z]])
        # print "end pos value is ", end_pos_it_val_x, end_pos_it_val_y, end_pos_it_val_z
        # print "r2 val ", r2
        # print "r2 end val ", r2_end
        var_temp1 = np.dot(omega_mat_start, r2)
        var_temp2 = np.dot(omega_mat_end, r2_end)
        # print "varmat temp1 ", var_mat1_temp1

        # Calculation of acceleration values based on the position value estimated
        start_mat_est_acc = Frame(Rotation.Quaternion(start_quat[0], start_quat[1], start_quat[2], start_quat[3]),
                           Vector(var_temp1[0], var_temp1[1], var_temp1[2]))
        end_mat_est_acc = Frame(Rotation.Quaternion(end_quat[0], end_quat[1], end_quat[2], end_quat[3]),
                         Vector(var_temp2[0], var_temp2[1], var_temp2[2]))
        twist_est_acc, twist_rot_est_acc = diff_calc_func(start_mat_est_acc, end_mat_est_acc, corrected_average_interval)

        print "Estimated acceleration value is ", twist_est_acc
        print "Actual acceleration val is ", twist_acc.x, twist_acc.y, twist_acc.z
        # For checking the acceleration values
        (check_pos_start, check_quat_start) = listener.lookupTransform(tracking_frame, reference_frame, start_time)
        (check_pos_end, check_quat_end) = listener.lookupTransform(tracking_frame, reference_frame, end_time)

        # Create homogeneous transformation matrix based on the position and quaternion
        rot_mat_ins_start = listener.fromTranslationRotation(check_pos_start, check_quat_start)
        rot_mat_ins_end = listener.fromTranslationRotation(check_pos_end, check_quat_end)

        # Creation of array of acceleration values
        arr_est_acc = np.array([[twist_est_acc.x], [twist_est_acc.y], [twist_est_acc.z]])
        arr_accel_val = np.array([[twist_acc.x], [twist_acc.y], [twist_acc.z]])
        # print " shap trial val ", trialval[0], trialval[1]

        # Create and transform the acceleration value into the desired frame
        mid_frame = np.dot(inv(rot_mat_ins_start[:3, :3]), rot_mat_ins_end[:3, :3])
        rot_est_acc_val = np.dot(mid_frame, np.reshape(arr_est_acc, (3, 1)))
        print "rotated estimated accel val ", rot_est_acc_val[0][0], rot_est_acc_val[1][0], rot_est_acc_val[2][0]

        # Normalize both acceleration vectors
        norm_val1 = np.linalg.norm(arr_accel_val)
        print "The normalized value of actual acceleration is ", norm_val1
        norm_val2 = np.linalg.norm(rot_est_acc_val)
        print "The normalized value of estimated acceleration is ", norm_val2

        # Publish the accelerometer acceleration value
        accel_pub = geometry_msgs.msg.WrenchStamped()
        accel_pub.header.frame_id = tracking_frame
        accel_pub.wrench.force.x = twist_acc.x
        accel_pub.wrench.force.y = twist_acc.y
        accel_pub.wrench.force.z = twist_acc.z

        # Publish the estimated acceleration value based on estimated position
        est_accel_pub = geometry_msgs.msg.WrenchStamped()
        est_accel_pub.header.frame_id = tracking_frame
        est_accel_pub.wrench.force.x = rot_est_acc_val[0][0]
        est_accel_pub.wrench.force.y = rot_est_acc_val[1][0]
        est_accel_pub.wrench.force.z = rot_est_acc_val[2][0]

        # Publish the difference between estimated acceleration value and value of accelerometer
        pos_diff_pub = geometry_msgs.msg.Vector3()
        pos_diff_pub.x = norm_val1- norm_val2

        # Publish the estimated position value of accelerometer
        x_pub = geometry_msgs.msg.Vector3()
        x_pub.x = x_val[0]
        x_pub.y = x_val[1]
        x_pub.z = x_val[2]

        # Publish all the values as ROS topics
        x_val_pub.publish(x_pub)
        position_diff_pub.publish(pos_diff_pub)
        est_acceleration_pub.publish(est_accel_pub)
        accelerometer_pub.publish(accel_pub)

        rate.sleep()