#ifndef TWIST_JOGGER_H_
#define TWIST_JOGGER_H_

#include <ros/ros.h>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/TwistStamped.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/Trigger.h>

#include <Eigen/Eigenvalues>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/GetPositionFK.h>
#include <moveit_msgs/GetPositionIK.h>

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

    void
    spin();

private:
    ros::NodeHandle nh_;

    /* Private node handle */
    ros::NodeHandle pnh_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    ros::Subscriber sub_twist_;

    ros::Subscriber sub_joint_state_;

    ros::Publisher pub_traj_;

    ros::ServiceClient srv_fk_;

    ros::ServiceClient srv_ik_;

    std::shared_ptr<MoveGroupInterface> move_group_;

    std::shared_ptr<robot_state::RobotState> kinematic_state_;

    const robot_state::JointModelGroup* joint_model_group_;

    robot_model_loader::RobotModelLoader model_loader_;

    /*
     * Current internal joint states
     * The reason why we treat the calculated (internal) joint state and the
     * actual joint values differently is because the actual joint values
     * fluctuate quite often. This makes it difficult when calculating the
     * next pose of the end-effector. It's better to assume that the
     * commanded joint values are perfectly executed, and base our next
     * end-effector pose calculation off of that assumption.
     *
     * However, we need to update to the actual joint values once in a while,
     * and that's done after internal_timeout_ seconds.
     *
     * The reason why we wait for a while before updating to the actual joint
     * values is because when repeating short intervals of jogging and
     * pausing, we saw some drifting of the end-effector due to the
     * fluctuation mentioned above.
    */
    std::mutex joints_curr_mutex_;
    sensor_msgs::JointState joints_curr_;

    /* Latest raw values received from via topic */
    std::mutex latest_twist_mutex_;
    geometry_msgs::TwistStamped latest_twist_;

    sensor_msgs::JointState latest_joint_state_;

    std::string move_group_name_;

    /**
     * frame_id to be used for the output JointTrajectory message
     */
    std::string base_frame_id_;

    /**
     * Name of the end-effector (the link to move)
     */
    std::string ee_link_name_;

    /**
     * How often to publish the JointTrajectory
     *
     * Note: Does not publish when the received twist is all zeros.
     */
    double publish_rate_;

    /**
     * How long the commanded one-point trajectory is (seconds)
     */
    double trajectory_duration_;

    /**
     * Limit movement of joints (rad and rad/s, respectively)
     */
    double max_ang_change_;
    double max_speed_;

    /**
     * Discard certain fields from JointState messages
     */
    bool discard_velocity_;
    bool discard_effort_;

    double fresh_twist_timeout_;

    /**
     * After this many seconds, discard the internal joint state and update
     * to the actual joint values.
     */
    double internal_timeout_;

    /* Private member methods */

    sensor_msgs::JointState
    get_next_joint_state(const geometry_msgs::TwistStamped& twist);

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

    bool
    is_below_joint_limits(const sensor_msgs::JointState &s1,
                          const sensor_msgs::JointState &s2);
    /**
     * Calculates the current pose of the end-effector from the
     * joint state
     *
     * This is done using the /compute_fk service.
     *
     * @param joints the current joint state of the robot
     * @param tgt_frame_id the frame of the end-effector
     * @return the pose of the end-effector in the cartesian coordinate
     */
    std::tuple<geometry_msgs::PoseStamped, bool>
    compute_fk(const sensor_msgs::JointState& joints,
               const std::string& fixed_frame_id,
               const std::string& tgt_frame_id);

    /**
     * Computes the inverse kinematics of a given pose
     *
     * @param pose the pose of the end-effector
     * @return the robot's joint state that achieves the given pose
     */
    std::tuple<sensor_msgs::JointState, bool>
    compute_ik(const geometry_msgs::PoseStamped& pose,
                   const sensor_msgs::JointState& joints_curr,
                   const std::string& tgt_frame_id);

    /**
     * Calculates the next pose of the end-effector
     *
     * The amount of movement is controlled by the given twist message and
     * also the dt value.
     *
     * @param curr_pose the current pose of the end-effector
     * @param twist the velocity command
     * @param dt the duration of movement
     */
    geometry_msgs::PoseStamped
    get_target_pose(const geometry_msgs::PoseStamped& curr_pose,
                    const geometry_msgs::TwistStamped& twist,
                    const double dt);

    /**
     * Resets the internal JointState to the given JointState message
     */
    void
    reset_to_actual_js(const sensor_msgs::JointState& msg);

    /**
     * Filters out the irrelevant joints from the JointState message
     *
     * discard_velocity and discard_effort ROS parameters control whether to
     * also discard the velocity and effort fields, respectively.
     *
     * Any joint that's not in the move_group will be discarded
     * @param msg
     * @return the filtered JointState message
     */
    sensor_msgs::JointState
    filter_joint_state(const sensor_msgs::JointState& msg);
};

#endif /* end of include guard */
