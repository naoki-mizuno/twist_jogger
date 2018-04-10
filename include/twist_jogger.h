#ifndef TWIST_JOGGER_H_
#define TWIST_JOGGER_H_

#include <ros/ros.h>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TwistStamped.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <sensor_msgs/JointState.h>

#include <Eigen/Eigenvalues>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

#include <mutex>
#include <string>
#include <thread>

class TwistJogger {
public:
    using Vector6d = Eigen::Matrix<double, 6, 1>;
    using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

    TwistJogger();

    ~TwistJogger() = default;

    TwistJogger(const TwistJogger& other) = delete;

    TwistJogger(TwistJogger&& other) = delete;

    TwistJogger&
    operator=(const TwistJogger& other) = delete;

    TwistJogger&
    operator=(TwistJogger&& other) = delete;

    void
    cb_twist(const geometry_msgs::TwistStamped& msg);

    void
    cb_js(const sensor_msgs::JointState& msg);

    /**
     * Runs in a loop
     */
    void
    spin();

    /**
     * Keeps calculating the joint angles from the twist message
     */
    void
    twist_to_joint_worker();

private:
    ros::NodeHandle nh_;

    /* Private node handle */
    ros::NodeHandle pnh_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    ros::Subscriber sub_twist_;

    ros::Subscriber sub_joint_state_;

    ros::Publisher pub_traj_;

    std::shared_ptr<MoveGroupInterface> move_group_;

    std::shared_ptr<robot_state::RobotState> kinematic_state_;

    const robot_state::JointModelGroup* joint_model_group_;

    robot_model_loader::RobotModelLoader model_loader_;

    /* Current joint state */
    std::mutex joints_curr_mutex_;
    sensor_msgs::JointState joints_curr_;

    /* Joint state after increment */
    sensor_msgs::JointState joints_next_;

    std::mutex latest_twist_mutex_;
    geometry_msgs::TwistStamped latest_twist_;

    std::mutex latest_jt_mutex_;
    trajectory_msgs::JointTrajectory latest_jt_;

    /**
     * frame_id to be used for the output JointTrajectory message
     */
    std::string planning_frame_id_;

    double stale_limit_;

    /**
     * How often to publish the JointTrajectory
     *
     * Does not publish when the received twist is all zeros.
     */
    double publish_rate_;

    /**
     * How long the trajectory is, in seconds
     */
    double trajectory_duration_;

    /**
     * How fine-grained the trajectory is, in seconds
     *
     * Used for the time_from_start field
     */
    double trajectory_resolution_;

    double threshold_cn_slow_down_;

    double threshold_cn_hard_stop_;

    /* Private member methods */

    trajectory_msgs::JointTrajectory
    get_joint_trajectory(const geometry_msgs::TwistStamped& twist);

    /**
     * Converts a JointState message to JointTrajectory
     * @param joints JointState message
     * @return an equivalent JointTrajectory message
     */
    trajectory_msgs::JointTrajectoryPoint
    js_to_jtp(const sensor_msgs::JointState& joints, unsigned point_id);

    sensor_msgs::JointState
    get_next_joint_state(const sensor_msgs::JointState& curr,
                         const Eigen::VectorXd& delta_theta,
                         const Eigen::VectorXd& omega);

    /**
     *
     * @param jacobian
     * @param vel_xyzrpy the velocity in xyzrpy
     * @return angular velocities of each joint
     */
    Eigen::VectorXd
    get_joint_omega(const Eigen::MatrixXd& jacobian,
                    const Vector6d& vel_xyzrpy);

    /**
     * @param jacobian
     * @return the pseudo-inverse matrix of the jacobian
     */
    Eigen::MatrixXd
    get_pseudo_inverse(const Eigen::MatrixXd& jacobian);

    /**
     * Converts a TwistStamped message to a Vector6d
     *
     * The vector's elements are in the following order:
     * 1. x
     * 2. y
     * 3. z
     * 4. roll
     * 5. pitch
     * 6. yaw
     *
     * @param twist
     * @return a Vector6d with the x, y, z, roll, pitch, yaw
     */
    Vector6d
    twist_to_vector6d(const geometry_msgs::TwistStamped& twist);

    geometry_msgs::TwistStamped
    transform_twist(const geometry_msgs::TwistStamped& twist,
                    const std::string& frame_id);

    /**
     * Checks whether the input twist message is all-zero
     *
     * @param twist the message to be checked
     * @return true if all fields in Twist is 0, false otherwise
     */
    bool
    is_zero_input(const geometry_msgs::TwistStamped& twist);

    /**
     * Adjust the velocity according to the condition number
     *
     * @param jacobian
     * @param vel_xyzrpy
     * @return the new cartesian velocities
     */
    Vector6d
    adjust_velocity(const Eigen::MatrixXd& jacobian,
                    const Vector6d& vel_xyzrpy);

    /**
     * Calculates the condition number from the jacobian
     *
     * The condition number indicates how close the current configuration
     * is to singularity. The higher the number, the closer it is to
     * singularity.
     *
     * @param jacobian
     * @return
     */
    double
    get_condition_number(const Eigen::MatrixXd& jacobian);
};

#endif /* end of include guard */
