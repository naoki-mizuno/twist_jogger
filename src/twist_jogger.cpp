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
    , joints_curr_mutex_{}
    , joints_curr_{}
    , latest_twist_mutex_{}
    , latest_twist_{}
    , base_link_name{}
    , ee_link_name_{}
    , publish_rate_{0}
    , trajectory_duration_{0}
    , discard_velocity_{true}
    , discard_effort_{true}
    , fresh_twist_timeout_{0}
    , internal_timeout_{0}
{
    std::string in_topic, out_topic, js_topic;
    pnh_.param("topic/twist", in_topic, std::string{"cmd_delta"});
    pnh_.param("topic/joint_states", js_topic, std::string{"joint_states"});
    pnh_.param("topic/trajectory", out_topic, std::string{"controller/command"});
    pnh_.param("move_group_name", move_group_name_, std::string{"move_group"});
    pnh_.param("tf_prefix", tf_prefix_, std::string{""});
    pnh_.param("base_link_name", base_link_name, std::string{"base_link"});
    pnh_.param("ee_link_name", ee_link_name_, std::string{"ee_link"});
    pnh_.param("publish_rate", publish_rate_, 100.0);
    pnh_.param("trajectory_duration", trajectory_duration_, 0.1);
    pnh_.param("joint_limits/speed", max_speed_, 1.0);
    pnh_.param("joint_limits/angle_change", max_ang_change_, 0.5);
    pnh_.param("discard_velocity", discard_velocity_, true);
    pnh_.param("discard_effort", discard_effort_, true);
    pnh_.param("timeout/fresh_twist", fresh_twist_timeout_, 0.2);
    pnh_.param("timeout/internal_joint_state", internal_timeout_, 3.0);

    move_group_ = std::make_shared<MoveGroupInterface>(move_group_name_);

    auto arm_model = model_loader_.getModel();
    kinematic_state_ = std::make_shared<robot_state::RobotState>(arm_model);
    kdl_parser::treeFromUrdfModel(*arm_model->getURDF(), robot_description_);
    ideal_rsp_ = std::make_shared<robot_state_publisher::RobotStatePublisher>(
            robot_description_
    );

    joint_model_group_ = arm_model->getJointModelGroup(move_group_name_);

    ROS_INFO_STREAM("Listening JointState on " << js_topic);
    auto msg = ros::topic::waitForMessage<sensor_msgs::JointState>(js_topic);
    joints_curr_ = filter_joint_state(*msg);
    latest_joint_state_ = *msg;
    reset_to_actual_js(latest_joint_state_);
    ROS_INFO_STREAM("Got first JointState message!");

    sub_twist_ = nh_.subscribe(in_topic, 1, &TwistJogger::cb_twist, this);
    sub_joint_state_ = nh_.subscribe(js_topic, 1, &TwistJogger::cb_js, this);
    pub_traj_ = nh_.advertise<trajectory_msgs::JointTrajectory>(out_topic, 1);
    srv_fk_ = nh_.serviceClient<moveit_msgs::GetPositionFK>("compute_fk");
    srv_ik_ = nh_.serviceClient<moveit_msgs::GetPositionIK>("compute_ik");

    // Actual joint state to ideal joint state
    const auto& root_joint = arm_model->getRootJoint();
    // Example: world
    auto root_link = root_joint->getChildLinkModel()->getName();
    // Example: erista/world
    auto actual_base_frame_id = append_prefix(tf_prefix_, root_link);
    // Example: ideal/world
    auto ideal_base_frame_id = append_prefix(IDEAL_TF_PREFIX, root_link);
    // Example: erista/ideal/world
    ideal_base_frame_id = append_prefix(tf_prefix_, ideal_base_frame_id);

    auto tform = geometry_msgs::TransformStamped{};
    tform.header.frame_id = actual_base_frame_id;
    tform.child_frame_id = ideal_base_frame_id;
    tform.transform.rotation.w = 1;
    tf_static_.sendTransform(tform);
}

void
TwistJogger::cb_twist(const geometry_msgs::TwistStamped& msg) {
    std::lock_guard<std::mutex> scoped_mutex{latest_twist_mutex_};
    latest_twist_ = msg;
}

void
TwistJogger::cb_js(const sensor_msgs::JointState& msg) {
    latest_joint_state_ = msg;
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

        auto twist_age = ros::Time::now() - latest_twist.header.stamp;
        if (is_zero_input(latest_twist)) {
            if (twist_age.toSec() > internal_timeout_) {
                reset_to_actual_js(latest_joint_state_);
            }
            continue;
        }

        // Consider the latest twist message stale
        if (twist_age.toSec() > fresh_twist_timeout_) {
            continue;
        }

        auto joints_next = get_next_joint_state(latest_twist);

        if (joints_next.position.empty()) {
            continue;
        }

        // Assume that the trajectory is perfectly executed
        joints_curr_mutex_.lock();
        joints_curr_ = joints_next;
        publish_ideal_pose(joints_curr_);
        joints_curr_mutex_.unlock();
        kinematic_state_->setVariableValues(joints_next);

        // Pack them into a JointTrajectory message
        trajectory_msgs::JointTrajectory jt;
        jt.header.frame_id = append_prefix(tf_prefix_, base_link_name);
        jt.joint_names = joints_next.name;
        trajectory_msgs::JointTrajectoryPoint point;
        point.positions = joints_next.position;
        point.velocities = joints_next.velocity;
        point.time_from_start = ros::Duration{trajectory_duration_};
        jt.points.push_back(point);

        pub_traj_.publish(jt);
    }
}

/* Private member methods */

sensor_msgs::JointState
TwistJogger::get_next_joint_state(const geometry_msgs::TwistStamped& twist) {
    auto joints_next = decltype(joints_curr_){};

    // Convenience variables
    joints_curr_mutex_.lock();
    const auto joints_curr = filter_joint_state(joints_curr_);
    joints_curr_mutex_.unlock();

    // Compute the current pose of the end-effector (in the base frame)
    geometry_msgs::PoseStamped curr_pose;
    bool success;
    const std::string base_frame_id = append_prefix(tf_prefix_,
                                                    base_link_name);
    std::tie(curr_pose, success) = compute_fk(joints_curr,
                                              base_frame_id,
                                              ee_link_name_);
    if (!success) {
        ROS_ERROR_STREAM("Failed to compute FK");
        return sensor_msgs::JointState{};
    }

    // Compute the next pose of the end-effector (could be ideal or actual)
    auto tgt_pose  = get_target_pose(curr_pose,
                                     twist,
                                     trajectory_duration_);
    tgt_pose.header.frame_id = base_frame_id;
    std::tie(joints_next, success) = compute_ik(tgt_pose,
                                                joints_curr,
                                                ee_link_name_);
    if (!success) {
        ROS_ERROR_STREAM("Failed to compute IK");
        return sensor_msgs::JointState{};
    }

    // A joint is going to move too much/too fast!
    if (!is_below_joint_limits(joints_curr, joints_next)) {
        ROS_ERROR_STREAM_THROTTLE(1, "A joint exceeds the movement limit!");
        return sensor_msgs::JointState{};
    }

    return joints_next;
}

std::tuple<geometry_msgs::PoseStamped, bool>
TwistJogger::compute_fk(const sensor_msgs::JointState& joints,
                        const std::string& fixed_frame_id,
                        const std::string& tgt_link_name) {
    auto fk = moveit_msgs::GetPositionFK{};
    fk.request.header.frame_id = fixed_frame_id;
    fk.request.header.stamp = ros::Time::now();
    fk.request.fk_link_names.clear();
    fk.request.fk_link_names.push_back(tgt_link_name);
    fk.request.robot_state.joint_state = joints;

    auto success = srv_fk_.call(fk);
    if (fk.response.error_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
        success = false;
    }
    return std::make_tuple(fk.response.pose_stamped.front(), success);
}

std::tuple<sensor_msgs::JointState, bool>
TwistJogger::compute_ik(const geometry_msgs::PoseStamped& pose,
                        const sensor_msgs::JointState& joints_curr,
                        const std::string& tgt_link_name) {
    auto ik = moveit_msgs::GetPositionIK{};
    auto& req = ik.request.ik_request;
    req.robot_state.joint_state = joints_curr;
    req.pose_stamped = pose;
    req.ik_link_name = tgt_link_name;
    req.group_name = move_group_name_;

    auto success = srv_ik_.call(ik);
    if (ik.response.error_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
        success = false;
    }

    // Note: compute_ik returns a JointState with all joints
    auto filtered_solution = filter_joint_state(ik.response.solution.joint_state);
    return std::make_tuple(filtered_solution, success);
}

geometry_msgs::PoseStamped
TwistJogger::get_target_pose(const geometry_msgs::PoseStamped& curr_pose,
                             const geometry_msgs::TwistStamped& twist,
                             const double dt) {
    // TODO: Use curr_pose.header.frame_id
    const std::string actual_fixed_frame = append_prefix(tf_prefix_,
                                                         base_link_name);
    const std::string actual_ee_frame = append_prefix(tf_prefix_,
                                                      ee_link_name_);
    const std::string ideal_prefix = append_prefix(tf_prefix_,
                                                   IDEAL_TF_PREFIX);
    const std::string ideal_fixed_frame = append_prefix(ideal_prefix,
                                                        base_link_name);
    const std::string ideal_ee_frame = append_prefix(ideal_prefix,
                                                     ee_link_name_);

    auto can_transform_ideal = tf_buffer_.canTransform(ideal_fixed_frame,
                                                       ideal_ee_frame,
                                                       ros::Time{0});
    auto can_transform_actual = tf_buffer_.canTransform(actual_fixed_frame,
                                                        actual_ee_frame,
                                                        ros::Time{0});

    auto ee_twist = twist;
    std::string fixed_frame;
    if (can_transform_ideal) {
        ee_twist.header.frame_id = ideal_ee_frame;
        fixed_frame = ideal_fixed_frame;
    }
    else if (can_transform_actual) {
        ee_twist.header.frame_id = actual_ee_frame;
        fixed_frame = actual_fixed_frame;
    }
    else {
        ROS_WARN_STREAM("No transform between twist frame and ee frame");
        return curr_pose;
    }
    // EE command in the curr_pose frame, so that twist can be added
    ee_twist = transform_twist(ee_twist, fixed_frame);

    auto tgt_pose = curr_pose;

    // Apply translation
    tgt_pose.pose.position.x += dt * ee_twist.twist.linear.x;
    tgt_pose.pose.position.y += dt * ee_twist.twist.linear.y;
    tgt_pose.pose.position.z += dt * ee_twist.twist.linear.z;

    // Apply rotation
    const double dx = dt * ee_twist.twist.angular.x;
    const double dy = dt * ee_twist.twist.angular.y;
    const double dz = dt * ee_twist.twist.angular.z;
    double angle = std::sqrt(dx * dx + dy * dy + dz * dz);
    auto axis = tf2::Vector3{0, 0, 1};
    if (angle < DBL_EPSILON) {
        angle = 0;
    }
    else {
        axis.setX(dx / angle);
        axis.setY(dy / angle);
        axis.setZ(dz / angle);
    }
    tf2::Quaternion q_curr, q_tgt, q_twist;
    tf2::convert(curr_pose.pose.orientation, q_curr);
    q_twist.setRotation(axis, angle);
    q_tgt = q_twist * q_curr;
    tf2::convert(q_tgt, tgt_pose.pose.orientation);

    // Back to curr_pose frame
    tf_buffer_.transform(tgt_pose, tgt_pose, curr_pose.header.frame_id);

    return tgt_pose;
}

void
TwistJogger::publish_ideal_pose(const sensor_msgs::JointState& ideal_joints) {
    std::map<std::string, double> joint_values;
    for (unsigned i = 0; i < ideal_joints.name.size(); i++) {
        const auto name = ideal_joints.name[i];
        const auto joint_val = ideal_joints.position[i];
        joint_values[name] = joint_val;
    }
    const auto ideal_tf_prefix = append_prefix(tf_prefix_, IDEAL_TF_PREFIX);
    ideal_rsp_->publishTransforms(joint_values,
                                  ros::Time::now(),
                                  ideal_tf_prefix);
    ideal_rsp_->publishFixedTransforms(ideal_tf_prefix, false);
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
TwistJogger::is_below_joint_limits(const sensor_msgs::JointState &s1,
                                   const sensor_msgs::JointState &s2) {
    for (unsigned i = 0; i < s1.position.size(); i++) {
        const auto s1p = s1.position[i];
        const auto s2p = s2.position[i];
        // rad
        const auto theta = std::abs(s1p - s2p);
        // rad/s
        const auto omega = theta / trajectory_duration_;

        if (max_ang_change_ > 0 && theta > max_ang_change_) {
            return false;
        }

        if (max_speed_ > 0 && omega > max_speed_) {
            return false;
        }
    }

    return true;
}

void
TwistJogger::reset_to_actual_js(const sensor_msgs::JointState& msg) {
    std::lock_guard<std::mutex> scoped_mutex{joints_curr_mutex_};
    joints_curr_ = filter_joint_state(msg);
}

sensor_msgs::JointState
TwistJogger::filter_joint_state(const sensor_msgs::JointState& msg) {
    if (msg.name.empty()) {
        return msg;
    }

    auto filtered_js = decltype(msg){};
    filtered_js.header = msg.header;

    for (const auto& name : joint_model_group_->getJointModelNames()) {
        auto it = std::find(msg.name.begin(), msg.name.end(), name);
        if (it == msg.name.end()) {
            // Joint doesn't belong to move_group
            continue;
        }
        auto idx = static_cast<unsigned>(std::distance(msg.name.begin(), it));
        filtered_js.name.push_back(msg.name[idx]);
        if (idx < msg.position.size()) {
            filtered_js.position.push_back(msg.position[idx]);
        }
        if (!discard_velocity_ && idx < msg.velocity.size()) {
            filtered_js.velocity.push_back(msg.velocity[idx]);
        }
        if (!discard_effort_ && idx < msg.effort.size()) {
            filtered_js.effort.push_back(msg.effort[idx]);
        }
    }
    return filtered_js;
}

geometry_msgs::TwistStamped
TwistJogger::transform_twist(const geometry_msgs::TwistStamped& twist,
                             const std::string& frame_id) {
    if (twist.header.frame_id == frame_id) {
        return twist;
    }
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

    new_twist.header.frame_id = frame_id;
    new_twist.twist.linear = new_lin.vector;
    new_twist.twist.angular = new_ang.vector;
    return new_twist;
}

std::string
TwistJogger::append_prefix(const std::string& prefix,
                           const std::string& frame_id) {
    if (prefix.empty()) {
        return frame_id;
    }
    auto p = prefix;
    auto f = frame_id;
    // Strip all leading and trailing slashes
    if (p.front() == '/') {
        p = p.substr(1);
    }
    if (p.back() == '/') {
        p = p.substr(0, p.size() - 1);
    }
    if (f.front() == '/') {
        f = f.substr(1);
    }
    if (f.back() == '/') {
        f = f.substr(0, f.size() - 1);
    }
    return p + "/" + f;
}