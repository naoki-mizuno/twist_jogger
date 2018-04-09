#include "twist_jogger.h"

TwistJogger::TwistJogger()
    : nh_{}
    , pnh_{"~"}
    , tf_buffer_{}
    , tf_listener_{tf_buffer_}
    , sub_twist_{}
    , sub_joint_state_{}
    , pub_traj_{}
    , move_group_{nullptr}
    , kinematic_state_{nullptr}
    , joint_model_group_{nullptr}
    , joints_curr_mutex_{}
    , joints_curr_{}
    , joints_next_{}
    , latest_twist_mutex_{}
    , latest_twist_{}
    , planning_frame_id_{}
    , publish_rate_{0}
    , trajectory_resolution_{0}
    , threshold_cn_slow_down_{0}
    , threshold_cn_hard_stop_{0}
{
    std::string in_topic, out_topic, js_topic, move_group_name;
    pnh_.param("twist_topic", in_topic, std::string{"cmd_delta"});
    pnh_.param("traj_topic", out_topic, std::string{"controller/command"});
    pnh_.param("joint_state_topic", js_topic, std::string{"joint_states"});
    pnh_.param("move_group_name", move_group_name, std::string{"move_group"});
    pnh_.param("planning_frame_id", planning_frame_id_, std::string{"world"});
    pnh_.param("publish_rate", publish_rate_, 100.0);
    pnh_.param("trajectory_resolution", trajectory_resolution_, 0.2);
    pnh_.param("thresholds/slow_down", threshold_cn_slow_down_, 12.0);
    pnh_.param("thresholds/hard_stop", threshold_cn_hard_stop_, 20.0);

    move_group_ = std::make_shared<MoveGroupInterface>(move_group_name);

    robot_model_loader::RobotModelLoader model_loader{"robot_description"};
    auto arm_model = model_loader.getModel();
    kinematic_state_ = std::make_shared<robot_state::RobotState>(arm_model);

    joint_model_group_ = arm_model->getJointModelGroup(move_group_name);

    ROS_INFO_STREAM("Listening to "
                    << js_topic
                    << " for JointState messages");
    ros::topic::waitForMessage<sensor_msgs::JointState>(js_topic);
    ROS_INFO_STREAM("Got JointState message!");

    sub_twist_ = nh_.subscribe(in_topic, 1, &TwistJogger::cb_twist, this);
    sub_joint_state_ = nh_.subscribe(js_topic, 1, &TwistJogger::cb_js, this);
    pub_traj_ = nh_.advertise<trajectory_msgs::JointTrajectory>(out_topic, 1);
}

void
TwistJogger::cb_twist(const geometry_msgs::TwistStamped& msg) {
    std::lock_guard<std::mutex> scoped_mutex{latest_twist_mutex_};
    latest_twist_ = msg;

    // TODO: Remove?
    if (is_zero_input(msg)) {
        return;
    }
    pub_traj_.publish(get_joint_trajectory(msg));
}

void
TwistJogger::cb_js(const sensor_msgs::JointState& msg) {
    sensor_msgs::JointState filtered_js;
    filtered_js.header = msg.header;

    for (const auto& name : joint_model_group_->getJointModelNames()) {
        auto it = std::find(msg.name.begin(), msg.name.end(), name);
        if (it == msg.name.end()) {
            // Joint doesn't belong to move_group
            continue;
        }
        auto idx = std::distance(msg.name.begin(), it);
        filtered_js.name.push_back(msg.name[idx]);
        filtered_js.position.push_back(msg.position[idx]);
        filtered_js.velocity.push_back(msg.velocity[idx]);
        filtered_js.effort.push_back(msg.effort[idx]);
    }
    std::lock_guard<std::mutex> scoped_mutex{joints_curr_mutex_};
    joints_curr_ = std::move(filtered_js);
}

void
TwistJogger::spin() {
    std::thread twist_to_joint(&TwistJogger::twist_to_joint_worker, this);
    twist_to_joint.detach();

    auto rate = ros::Rate{publish_rate_};
    while (ros::ok()) {
        rate.sleep();

        // TODO: Add with stale limit

        std::lock_guard<std::mutex> scoped_mutex{latest_jt_mutex_};
        pub_traj_.publish(latest_jt_);
    }
}

void
TwistJogger::twist_to_joint_worker() {
    // Spin really fast, but not too fast
    auto rate = ros::Rate{1000};
    while (ros::ok()) {
        rate.sleep();

        latest_twist_mutex_.lock();
        auto latest_twist = geometry_msgs::TwistStamped{latest_twist_};
        latest_twist_mutex_.unlock();

        if (is_zero_input(latest_twist)) {
            continue;
        }

        latest_jt_mutex_.lock();
        latest_jt_ = get_joint_trajectory(latest_twist);
        latest_jt_mutex_.unlock();
    }
}

/* Private member methods */

trajectory_msgs::JointTrajectory
TwistJogger::get_joint_trajectory(const geometry_msgs::TwistStamped& twist) {
    // Cartesian velocities
    auto vel_xyzrpy = twist_to_vector6d(twist);
    auto jacobian = kinematic_state_->getJacobian(joint_model_group_);
    // Take singularity into account
    vel_xyzrpy = adjust_velocity(jacobian, vel_xyzrpy);
    // Angular velocities for each joint
    auto omega = get_joint_omega(jacobian, vel_xyzrpy);
    // Positions for each joint
    auto delta_theta = omega * trajectory_resolution_;

    // By now the joints_curr_ should have been updated in the callback
    joints_curr_mutex_.lock();
    joints_next_ = get_next_joint_state(joints_curr_, delta_theta, omega);
    joints_curr_mutex_.unlock();
    kinematic_state_->setVariableValues(joints_next_);

    return js_to_jt(joints_next_);
}

trajectory_msgs::JointTrajectory
TwistJogger::js_to_jt(const sensor_msgs::JointState& joints) {
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions = joints.position;
    point.velocities = joints.velocity;
    // point.effort = joints.effort;
    point.time_from_start = ros::Duration{trajectory_resolution_};

    trajectory_msgs::JointTrajectory jt;
    jt.header.frame_id = planning_frame_id_;
    jt.header.stamp = ros::Time::now();
    jt.joint_names = joints.name;
    jt.points.push_back(point);

    return jt;
}

sensor_msgs::JointState
TwistJogger::get_next_joint_state(const sensor_msgs::JointState& curr,
                                  const Eigen::VectorXd& delta_theta,
                                  const Eigen::VectorXd& omega) {
    auto next = curr;
    for (std::size_t i = 0; i < next.name.size(); i++) {
        next.position[i] += delta_theta[i];
        next.velocity[i] = omega[i];
    }
    return next;
}

Eigen::VectorXd
TwistJogger::get_joint_omega(const Eigen::MatrixXd& jacobian,
                             const TwistJogger::Vector6d& vel_xyzrpy) {
    return get_pseudo_inverse(jacobian) * vel_xyzrpy;
}

Eigen::MatrixXd
TwistJogger::get_pseudo_inverse(const Eigen::MatrixXd& jacobian) {
    Eigen::MatrixXd transpose = jacobian.transpose();
    return transpose * (jacobian * transpose).inverse();
}

TwistJogger::Vector6d
TwistJogger::twist_to_vector6d(const geometry_msgs::TwistStamped& twist) {
    auto transformed_twist = transform_twist(twist, planning_frame_id_);

    Vector6d vec;
    vec[0] = transformed_twist.twist.linear.x;
    vec[1] = transformed_twist.twist.linear.y;
    vec[2] = transformed_twist.twist.linear.z;
    vec[3] = transformed_twist.twist.angular.x;
    vec[4] = transformed_twist.twist.angular.y;
    vec[5] = transformed_twist.twist.angular.z;
    return vec;
}

geometry_msgs::TwistStamped
TwistJogger::transform_twist(const geometry_msgs::TwistStamped& twist,
                             const std::string& frame_id) {
    // Note: Why there is no transformTwist in the TF API
    // http://wiki.ros.org/tf/Reviews/2010-03-12_API_Review
    geometry_msgs::Vector3Stamped lin, ang, new_lin, new_ang;
    lin.header = twist.header;
    lin.vector = twist.twist.linear;
    ang.header = twist.header;
    ang.vector = twist.twist.angular;

    const auto timeout = ros::Duration{0.2};
    tf_buffer_.transform(lin, new_lin, frame_id, timeout);
    tf_buffer_.transform(ang, new_ang, frame_id, timeout);

    auto new_twist = geometry_msgs::TwistStamped{};
    new_twist.header = new_lin.header;
    new_twist.twist.linear = new_lin.vector;
    new_twist.twist.angular = new_ang.vector;
    return new_twist;
}

bool
TwistJogger::is_zero_input(const geometry_msgs::TwistStamped& twist) {
    return twist.twist.linear.x == 0
           && twist.twist.linear.y == 0
           && twist.twist.linear.z == 0
           && twist.twist.angular.x == 0
           && twist.twist.angular.y == 0
           && twist.twist.angular.z == 0;
}

TwistJogger::Vector6d
TwistJogger::adjust_velocity(const Eigen::MatrixXd& jacobian,
                             const TwistJogger::Vector6d& vel_xyzrpy) {
    auto cond_num = get_condition_number(jacobian);

    auto new_delta = vel_xyzrpy;
    if (cond_num > threshold_cn_hard_stop_) {
        // TODO: Parameterize?
        new_delta *= 0.7;
    }
    else if (cond_num > threshold_cn_slow_down_) {
        new_delta *= 0.3;
    }
    return vel_xyzrpy;
}

double
TwistJogger::get_condition_number(const Eigen::MatrixXd& jacobian) {
    Eigen::VectorXd eigen_vector = jacobian.eigenvalues().cwiseAbs();

    double min = eigen_vector.minCoeff();
    double max = eigen_vector.maxCoeff();

    double condition_number = max / min;
    return condition_number;
}