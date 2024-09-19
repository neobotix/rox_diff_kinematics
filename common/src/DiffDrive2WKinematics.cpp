/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Neobotix GmbH
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Neobotix nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include "../include/DiffDrive2WKinematics.h"

#include <geometry_msgs/msg/vector3.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


DiffDrive2WKinematics::DiffDrive2WKinematics()
{
}

void DiffDrive2WKinematics::execForwKin(const sensor_msgs::msg::JointState::SharedPtr js, nav_msgs::msg::Odometry& odom)
{
    // compute current velocities
    const double vel_x = 0.5 * (js->velocity[0] - js->velocity[1]) * (m_dDiam * 0.5);

    const double yaw_rate = -1 * (js->velocity[0] + js->velocity[1]) * (m_dDiam * 0.5) / m_dAxisLength;

    if(!m_curr_odom.header.stamp.sec == 0)
    {
        const double t_js = (js->header.stamp.sec + (js->header.stamp.nanosec/1e9));

        const double t_odom = ( m_curr_odom.header.stamp.sec + (m_curr_odom.header.stamp.nanosec/1e9));

        const double dt = t_js - t_odom;

        // compute second order midpoint velocities
        const double vel_x_mid = 0.5 * (vel_x + m_curr_odom.twist.twist.linear.x);
        const double yaw_rate_mid = 0.5 * (yaw_rate + m_curr_odom.twist.twist.angular.z);

        // compute midpoint yaw angle
        const double phi_mid = m_phiAbs + yaw_rate_mid * dt * 0.5;

        // integrate position using midpoint velocities and yaw angle
        m_curr_odom.pose.pose.position.x += vel_x_mid * dt * cos(phi_mid);
        m_curr_odom.pose.pose.position.y += vel_x_mid * dt * sin(phi_mid);
        m_curr_odom.pose.pose.position.z = 0;

        // integrate yaw angle using midpoint yaw rate
        m_phiAbs += yaw_rate_mid * dt;

        // Converting to quaternions
        tf2::Quaternion q;
        geometry_msgs::msg::Quaternion quat_msg;
        q.setRPY(0, 0, m_phiAbs);
        quat_msg = tf2::toMsg(q);
        m_curr_odom.pose.pose.orientation.x = quat_msg.x;
        m_curr_odom.pose.pose.orientation.y = quat_msg.y;
        m_curr_odom.pose.pose.orientation.z = quat_msg.z;
        m_curr_odom.pose.pose.orientation.w = quat_msg.w;
    }

    // update timestamp and velocities last
    
    m_curr_odom.header.stamp = js->header.stamp;
    m_curr_odom.header.frame_id = odom.header.frame_id;
    m_curr_odom.child_frame_id = odom.child_frame_id;

    m_curr_odom.twist.twist.linear.x = vel_x;
    m_curr_odom.twist.twist.linear.y = 0;
    m_curr_odom.twist.twist.linear.z = 0;
    m_curr_odom.twist.twist.angular.x = 0;
    m_curr_odom.twist.twist.angular.y = 0;
    m_curr_odom.twist.twist.angular.z = yaw_rate;

    // return data
    odom = m_curr_odom;
}

void DiffDrive2WKinematics::execInvKin(const geometry_msgs::msg::Twist::SharedPtr twist, trajectory_msgs::msg::JointTrajectory& traj)
{
    traj.joint_names.clear();
    traj.points.clear();

    //angular velocity in rad
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.velocities.resize(4);

    // w1:
    traj.joint_names.push_back("wheel_left_joint");
    point.velocities[0] = (twist->linear.x - (twist->angular.z * m_dAxisLength) / 2) * 2 / m_dDiam;

    // w2:
    traj.joint_names.push_back("wheel_right_joint");
    point.velocities[1] = -(twist->linear.x + (twist->angular.z * m_dAxisLength) / 2) * 2 / m_dDiam;

    traj.points.push_back(point);
}

void DiffDrive2WKinematics::setAxisLength(double dLength)
{
    m_dAxisLength = dLength;
}

void DiffDrive2WKinematics::setWheelDiameter(double dDiam)
{
    m_dDiam = dDiam;
}

