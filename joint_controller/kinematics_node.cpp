#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <Eigen/Dense>
#include <cmath>
#include <string>
#include <vector>

class KinematicsNode : public rclcpp::Node {
public:
  KinematicsNode() : Node("kinematics_node") {
    // --- Subscribers ---
    sub_end_pos_ = this->create_subscription<geometry_msgs::msg::Vector3>(
      "target/end_pos", 10, [this](const geometry_msgs::msg::Vector3 &msg){
        vp_c_re_ = Eigen::Vector3d(msg.x, msg.y, msg.z);
      });

    sub_body_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "target/body_vel", 10, [this](const geometry_msgs::msg::Twist &msg){
        target_vel_body_(0) = msg.linear.x;
        target_vel_body_(1) = msg.linear.y;
        target_vel_body_(2) = msg.linear.z;
        target_vel_body_(3) = msg.angular.x;
        target_vel_body_(4) = msg.angular.y;
        target_vel_body_(5) = msg.angular.z;
      });

    sub_end_vel_ = this->create_subscription<geometry_msgs::msg::Vector3>(
      "target/end_vel", 10, [this](const geometry_msgs::msg::Vector3 &msg){
        target_vel_end_ = Eigen::Vector3d(msg.x, msg.y, msg.z);
      });

    // --- Publisher ---
    pub_joint_state_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    // --- Timer (100 Hz) ---
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&KinematicsNode::step, this));

    // --- Defaults (元コードの値) ---
    vp_c_re_ << 0.1, -0.075 + -0.055 + 0.15, -0.16 + -0.173 + 0.1;
    target_vel_body_.setZero(); target_vel_body_(0)=0.1; target_vel_body_(1)=0.1;
    target_vel_end_  << 0.1, 0.1, 0.2;

    // Links
    vp_c_rwl_  << -0.091, -0.075,  0.0;
    vp_rwl_rwp_<<  0.091, -0.055,  0.0;
    vp_rwp_rkp_<<  0.0  ,  0.0  , -0.16;
    vp_rkp_re_ <<  0.0  ,  0.0  , -0.173;

    joint_names_ = {"rwl", "rwp", "rkp"};
  }

private:
  // ===== Core step =====
  void step() {
    using Eigen::Matrix4d; using Eigen::Vector4d; using Eigen::Vector3d; using Eigen::MatrixXd;

    // ---------- IK ----------
    // theta_rwl
    double theta_rwl = M_PI/2.0 - std::atan2(std::abs(vp_c_re_.z()),
                        vp_c_re_.y() - (vp_c_rwl_.y() + vp_rwl_rwp_.y()));

    // T_wr
    Matrix4d T_wr;
    T_wr << 1, 0, 0, -0.091,
            0, std::cos(theta_rwl), -std::sin(theta_rwl), -0.075,
            0, std::sin(theta_rwl),  std::cos(theta_rwl),  0.0,
            0, 0, 0, 1.0;

    // rwp位置（同次変換）
    Vector4d vv(vp_rwl_rwp_.x(), vp_rwl_rwp_.y(), vp_rwl_rwp_.z(), 1.0);
    Vector4d vp_c_rwp_h = T_wr * vv;

    // vr = (target) - (current to rwp)
    Vector3d vr = vp_c_re_ - Vector3d(vp_c_rwp_h(0), vp_c_rwp_h(1), vp_c_rwp_h(2));

    double a = std::abs(vp_rwp_rkp_.z());
    double b = std::abs(vp_rkp_re_.z());
    double c = std::sqrt(vr.squaredNorm());

    double theta_rkp = -std::acos((a*a + b*b - c*c) / (2.0 * a * b)) + M_PI;

    double foo = std::atan2(vr.x(), std::sqrt(vr.y()*vr.y() + vr.z()*vr.z())); // vs Z
    double bar = std::asin((b * std::sin(M_PI - theta_rkp)) / c);               // hip pitch 内角
    double theta_rwp = -(foo + bar);

    // ---------- FK ----------
    Matrix4d T_wp;
    T_wp <<  std::cos(theta_rwp), 0,  std::sin(theta_rwp),  0.091,
             0,                   1,  0,                   -0.055,
            -std::sin(theta_rwp), 0,  std::cos(theta_rwp),  0.0,
             0,                   0,  0,                    1.0;

    Matrix4d T_kp;
    T_kp <<  std::cos(theta_rkp), 0,  std::sin(theta_rkp),  0.0,
             0,                   1,  0,                    0.0,
            -std::sin(theta_rkp), 0,  std::cos(theta_rkp), -0.16,
             0,                   0,  0,                    1.0;
    Vector4d T_e(0.0, 0.0, -0.173, 1.0);

    Matrix4d fk_wr = T_wr;
    Matrix4d fk_wp = T_wr * T_wp;
    Matrix4d fk_kp = T_wr * T_wp * T_kp;
    Vector4d fk_e  = T_wr * T_wp * T_kp * T_e;

    // ---------- Jacobian (3x3) ----------
    Eigen::Matrix<double,3,3> J; J.setZero();
    Vector3d omega_rwl(1,0,0);
    Vector3d omega_rwp = T_wr.block<3,3>(0,0) * Vector3d(0,1,0);
    Vector3d omega_rkp = (T_wr*T_wp).block<3,3>(0,0) * Vector3d(0,1,0);

    Vector3d p_rwl = fk_wr.block<3,1>(0,3);
    Vector3d p_rwp = fk_wp.block<3,1>(0,3);
    Vector3d p_rkp = fk_kp.block<3,1>(0,3);
    Vector3d p_e   = fk_e.block<3,1>(0,0);

    J.block<3,1>(0,0) = omega_rwl.cross(p_e - p_rwl);
    J.block<3,1>(0,1) = omega_rwp.cross(p_e - p_rwp);
    J.block<3,1>(0,2) = omega_rkp.cross(p_e - p_rkp);

    // ---------- Body & end velocities ----------
    Eigen::Vector3d calc_vel_body =
      target_vel_body_.block<3,1>(0,0) + target_vel_body_.block<3,1>(3,0).cross(p_e);
    Eigen::Vector3d calc_vel_end  = target_vel_end_ - calc_vel_body;

    // ---------- q_dot (pseudo-inverse) ----------
    // SVD-based pseudo inverse (robust)
    Eigen::JacobiSVD<Eigen::Matrix<double,3,3>> svd(J, Eigen::ComputeFullU | Eigen::ComputeFullV);
    const auto &S = svd.singularValues();
    Eigen::Matrix3d S_inv = Eigen::Matrix3d::Zero();
    double eps = 1e-8;
    for (int i=0;i<3;i++) if (S(i) > eps) S_inv(i,i) = 1.0 / S(i);
    Eigen::Matrix3d J_pinv = svd.matrixV() * S_inv * svd.matrixU().transpose();

    Eigen::Vector3d q_dot = J_pinv * calc_vel_end;

    // ---------- Publish JointState ----------
    sensor_msgs::msg::JointState js;
    js.header.stamp = this->get_clock()->now();
    js.name = joint_names_;
    js.position = {theta_rwl, theta_rwp, theta_rkp}; // rad
    js.velocity = {q_dot(0), q_dot(1), q_dot(2)};   // rad/s
    pub_joint_state_->publish(js);
  }

  // --- ROS ---
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sub_end_pos_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr   sub_body_vel_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sub_end_vel_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr   pub_joint_state_;
  rclcpp::TimerBase::SharedPtr timer_;

  // --- Params / States ---
  Eigen::Vector3d vp_c_re_;
  Eigen::Matrix<double,6,1> target_vel_body_;
  Eigen::Vector3d target_vel_end_;
  Eigen::Vector3d vp_c_rwl_, vp_rwl_rwp_, vp_rwp_rkp_, vp_rkp_re_;
  std::vector<std::string> joint_names_;
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KinematicsNode>());
  rclcpp::shutdown();
  return 0;
}
