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
    , model_loader_{"robot_description"}
    , got_first_js_{false}
    , joints_curr_mutex_{}
    , joints_curr_{}
    , joints_next_{}
    , latest_twist_mutex_{}
    , latest_twist_{}
    , planning_frame_id_{}
    , stale_limit_{0}
    , js_divergence_{0}
    , publish_rate_{0}
    , trajectory_duration_{0}
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
    pnh_.param("stale_limit", stale_limit_, 0.2);
    pnh_.param("joint_state_divergence", js_divergence_, 0.05);
    pnh_.param("publish_rate", publish_rate_, 100.0);
    pnh_.param("trajectory/duration", trajectory_duration_, 0.5);
    pnh_.param("trajectory/resolution", trajectory_resolution_, 0.1);
    pnh_.param("thresholds/max_speed", threshold_max_speed_, 0.0);
    pnh_.param("singularity/slow_down/cn", threshold_cn_slow_down_, 12.0);
    pnh_.param("singularity/hard_stop/cn", threshold_cn_hard_stop_, 20.0);
    pnh_.param("singularity/slow_down/scale", slow_down_scale_, 0.3);
    pnh_.param("singularity/hard_stop/scale", hard_stop_scale_, 0.0);

    move_group_ = std::make_shared<MoveGroupInterface>(move_group_name);

    auto arm_model = model_loader_.getModel();
    kinematic_state_ = std::make_shared<robot_state::RobotState>(arm_model);

    joint_model_group_ = arm_model->getJointModelGroup(move_group_name);

    ROS_INFO_STREAM("Listening JointState on " << js_topic);
    ros::topic::waitForMessage<sensor_msgs::JointState>(js_topic);
    ROS_INFO_STREAM("Got JointState message!");

    sub_twist_ = nh_.subscribe(in_topic, 1, &TwistJogger::cb_twist, this);
    sub_joint_state_ = nh_.subscribe(js_topic, 1, &TwistJogger::cb_js, this);
    pub_traj_ = nh_.advertise<trajectory_msgs::JointTrajectory>(out_topic, 1);
    pub_cond_ = nh_.advertise<std_msgs::Float64>("condition_number", 1);
}

void
TwistJogger::cb_twist(const geometry_msgs::TwistStamped& msg) {
    std::lock_guard<std::mutex> scoped_mutex{latest_twist_mutex_};
    latest_twist_ = msg;
}

void
TwistJogger::cb_js(const sensor_msgs::JointState& msg) {
    if (!got_first_js_) {
        std::lock_guard<std::mutex> scoped_mutex{joints_curr_mutex_};
        joints_curr_ = msg;
        got_first_js_ = true;
    }
    auto divergence = get_divergence(msg);
    ROS_DEBUG_STREAM("Divergence: " << divergence);
    auto stationary = is_stationary(msg);
    if (!stationary) {
        if (divergence < js_divergence_) {
            return;
        }

        ROS_WARN_STREAM("Divergence detected between actual and calculated"
                        " positions. Updating to actual joint state."
                        " Divergence was " << divergence);
    }
    ROS_DEBUG_STREAM_THROTTLE(5, "Joints at rest: Updating internal "
                              "joint state to actual values");

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
    auto rate = ros::Rate{publish_rate_};
    while (ros::ok()) {
        rate.sleep();
        ros::spinOnce();

        latest_twist_mutex_.lock();
        auto latest_twist = geometry_msgs::TwistStamped{latest_twist_};
        latest_twist_mutex_.unlock();

        if (is_zero_input(latest_twist)) {
            continue;
        }

        auto t = ros::Time::now() - latest_twist_.header.stamp;
        if (t.toSec() > stale_limit_) {
            continue;
        }

        latest_jt_ = get_joint_trajectory(latest_twist);
        pub_traj_.publish(latest_jt_);
    }
}

/* Private member methods */

trajectory_msgs::JointTrajectory
TwistJogger::get_joint_trajectory(const geometry_msgs::TwistStamped& twist) {
    trajectory_msgs::JointTrajectory jt;
    jt.header.frame_id = planning_frame_id_;
    joints_curr_mutex_.lock();
    jt.joint_names = joints_curr_.name;
    joints_curr_mutex_.unlock();

    // Cartesian velocities
    auto vel_xyzrpy = twist_to_vector6d(twist);

    auto num_points = static_cast<unsigned>(
        trajectory_duration_ / trajectory_resolution_
    );
    for (unsigned i = 0; i < num_points; i++) {
        auto jacobian = kinematic_state_->getJacobian(joint_model_group_);
        // Take singularity into account
        vel_xyzrpy = adjust_velocity(jacobian, vel_xyzrpy);
        // Angular velocities for each joint
        auto omega = get_joint_omega(jacobian, vel_xyzrpy);
        if (!is_below_speed_limit(omega)) {
            omega = TwistJogger::Vector6d{};
        }
        // Positions for each joint
        auto delta_theta = omega * trajectory_resolution_;

        // By now the joints_curr_ should have been updated in the callback
        joints_curr_mutex_.lock();
        joints_next_ = get_next_joint_state(joints_curr_, delta_theta, omega);
        joints_curr_ = joints_next_;
        joints_curr_mutex_.unlock();
        kinematic_state_->setVariableValues(joints_next_);

        auto point = js_to_jtp(joints_next_, i + 1);
        jt.points.push_back(point);
    }

    return jt;
}

trajectory_msgs::JointTrajectoryPoint
TwistJogger::js_to_jtp(const sensor_msgs::JointState& joints,
                       const unsigned point_id) {
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions = joints.position;
    point.velocities = joints.velocity;
    // point.effort = joints.effort;
    point.time_from_start = ros::Duration{point_id * trajectory_resolution_};

    return point;
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

    TwistJogger::Vector6d vec;
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
    geometry_msgs::Vector3Stamped lin, ang, new_lin, new_ang;
    lin.header = twist.header;
    lin.vector = twist.twist.linear;
    ang.header = twist.header;
    ang.vector = twist.twist.angular;

    auto new_twist = geometry_msgs::TwistStamped{};
    new_twist.header = new_lin.header;

    const auto timeout = ros::Duration{0.2};
    try {
        tf_buffer_.transform(lin,
                             new_lin,
                             frame_id,
                             ros::Time{0},
                             lin.header.frame_id,
                             timeout);
        tf_buffer_.transform(ang,
                             new_ang,
                             frame_id,
                             ros::Time{0},
                             ang.header.frame_id,
                             timeout);
    }
    catch (const tf2::ExtrapolationException& e) {
        ROS_WARN_STREAM(e.what() << std::endl << "Returning zeros.");
        return new_twist;
    }

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

bool
TwistJogger::is_stationary(const sensor_msgs::JointState& msg) {
    auto gt = boost::bind(std::equal_to<double>{}, _1, 0);
    return std::all_of(msg.velocity.begin(), msg.velocity.end(), gt);
}

TwistJogger::Vector6d
TwistJogger::adjust_velocity(const Eigen::MatrixXd& jacobian,
                             const TwistJogger::Vector6d& vel_xyzrpy) {
    auto cond_num = get_condition_number(jacobian);

    auto cn = std_msgs::Float64{};
    cn.data = cond_num;
    pub_cond_.publish(cn);

    auto new_delta = vel_xyzrpy;
    if (cond_num > threshold_cn_hard_stop_) {
        ROS_ERROR_STREAM_THROTTLE(2, "Approaching singularity: stopping! "
                                     << cond_num);
        new_delta *= hard_stop_scale_;
    }
    else if (cond_num > threshold_cn_slow_down_) {
        ROS_WARN_STREAM_THROTTLE(2, "Approaching singularity: slowing down! "
                                    << cond_num);
        new_delta *= slow_down_scale_;
    }
    return new_delta;
}

bool
TwistJogger::is_below_speed_limit(const TwistJogger::Vector6d& omega) {
    const auto& bounds = joint_model_group_->getActiveJointModelsBounds();
    for (unsigned i = 0; i < bounds.size(); i++) {
        // For serial links, bounds[i].size() == 1
        auto vel_limit = bounds[i]->front().max_velocity_;
        if (threshold_max_speed_ != 0 && vel_limit > threshold_max_speed_) {
            vel_limit = threshold_max_speed_;
        }
        if (omega[i] > vel_limit) {
            ROS_WARN_STREAM_THROTTLE(2, "Command exceeds velocity limit!");
            return false;
        }
    }

    return true;
}

double
TwistJogger::get_condition_number(const Eigen::MatrixXd& jacobian) {
    Eigen::VectorXd eigen_vector = jacobian.eigenvalues().cwiseAbs();

    double min = eigen_vector.minCoeff();
    double max = eigen_vector.maxCoeff();

    double condition_number = max / min;
    return condition_number;
}

double
TwistJogger::get_divergence(const sensor_msgs::JointState& msg) {
    auto ee_link_name = move_group_->getEndEffectorLink();

    // Calculated pose from commanded joint values
    joints_curr_mutex_.lock();
    kinematic_state_->setVariableValues(joints_curr_);
    joints_curr_mutex_.unlock();
    auto e = kinematic_state_->getGlobalLinkTransform(ee_link_name);

    // Actual pose (obtained from the JointState message)
    kinematic_state_->setVariableValues(msg);
    auto c = kinematic_state_->getGlobalLinkTransform(ee_link_name);

    auto dx = c.translation()[0] - e.translation()[0];
    auto dy = c.translation()[1] - e.translation()[1];
    auto dz = c.translation()[2] - e.translation()[2];
    auto dist_diff = std::sqrt(dx * dx + dy * dy + dz * dz);

    return dist_diff;
}
