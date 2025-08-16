#include <Eigen/Dense>
#include <iostream>
#include <cmath>
#include <iomanip>

int main() {
  using namespace Eigen;
  using std::cout;
  using std::endl;

  auto rad2deg = [](double r){ return r * 180.0 / M_PI; };

  // 入力

  // 重心の目標位置を座標系の原点に変えるような処理もあったほうがいい？

  // 目標位置（足先）
  Vector3d vp_c_re(
    0.0,
    -0.075 + -0.055,
    -0.16 + -0.173
  );
  cout << "vp_c_re: " << vp_c_re.transpose() << endl;

  // 目標速度（重心）
  Vector6d target_vel_body(
    0,0,0,0,0,0
  );
  

  // 目標速度（足先）
  Vector6d target_vel_end(
    0,0,0,0,0,0
  );

  // Joint angles
  double theta_rwl = 0.0; // Right Waist Roll
  double theta_rwp = 0.0; // Right Waist Pitch
  double theta_rkp = 0.0; // Right Knee Pitch

  // ---------- IK ----------
  // theta_rwl
  theta_rwl = M_PI/2.0 - std::atan2(std::abs(vp_c_re.z()),
                                    vp_c_re.y() - (vp_c_rwl.y() + vp_rwl_rwp.y()));
  cout << "theta_rwl: " << rad2deg(theta_rwl) << endl;

  // T_wr
  Eigen::Matrix4d T_wr;
  T_wr << 1, 0,               0,               -0.091,
          0, std::cos(theta_rwl), -std::sin(theta_rwl), -0.075,
          0, std::sin(theta_rwl),  std::cos(theta_rwl),  0.0,
          0, 0,               0,                1.0;

  // vp_c_rwp = T_wr * [vp_rwl_rwp; 1]
  Vector4d vv(vp_rwl_rwp.x(), vp_rwl_rwp.y(), vp_rwl_rwp.z(), 1.0);
  Vector4d vp_c_rwp_h = T_wr * vv;
//  cout << "vp_c_rwp: " << vp_c_rwp_h.transpose() << endl;

  // vr = (target) - (current to rwp)
  Vector3d vr = vp_c_re - Vector3d(vp_c_rwp_h(0), vp_c_rwp_h(1), vp_c_rwp_h(2));
//  cout << "vr: " << vr.transpose() << endl;

  double a = std::abs(vp_rwp_rkp.z());
  double b = std::abs(vp_rkp_re.z());
  double c = std::sqrt(vr.squaredNorm());
//  cout << "c: " << c << endl;

  theta_rkp = -std::acos((a*a + b*b - c*c) / (2.0 * a * b)) + M_PI;

  double foo = std::atan2(vr.x(), std::sqrt(vr.y()*vr.y() + vr.z()*vr.z())); // angle vs Z
  double bar = std::asin((b * std::sin(M_PI - theta_rkp)) / c);               // inner angle at hip pitch
//  cout << "foo: " << rad2deg(foo) << endl;
//  cout << "bar: " << rad2deg(bar) << endl;

  theta_rwp = -(foo + bar); // direction correction
  cout << std::fixed << std::setprecision(6);
  cout << "theta_rwp: " << rad2deg(theta_rwp) << endl;
  cout << "theta_rkp: " << rad2deg(theta_rkp) << endl;

  
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
  cout << "fk_e: " << fk_e.transpose() << endl;
  
  // ---------- Jacobian ----------
  Matrix<double, 6, 3> J;
  J.setZero();

  // omega の定義（回転軸ベクトル）
  Vector3d omega_rwl(1,0,0);
  Vector3d omega_rwp = T_wr.block<3,3>(0,0) * Vector3d(0,1,0);
  Vector3d omega_rkp = (T_wr*T_wp).block<3,3>(0,0) * Vector3d(0,1,0);;

  // p の定義（位置ベクトル）
  Vector3d p_rwl = fk_wr.block<3, 1>(0, 3);
  Vector3d p_rwp = fk_wp.block<3, 1>(0, 3);
  Vector3d p_rkp = fk_kp.block<3, 1>(0, 3);
  Vector3d p_e   = fk_e.block<3, 1>(0, 0);

  // 各列の計算: [ z × (pe - pi) ; z ]
  // 関節0
  J.block<3,1>(0,0) = omega_rwl.cross(p_e - p_rwl);
  J.block<3,1>(3,0) = omega_rwl;

  // 関節1
  J.block<3,1>(0,1) = omega_rwp.cross(p_e - p_rwp);
  J.block<3,1>(3,1) = omega_rwp;

  // 関節2
  J.block<3,1>(0,2) = omega_rkp.cross(p_e - p_rkp);
  J.block<3,1>(3,2) = omega_rkp;

  // Jを出力
  Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
  std::cout << "J = \n" << J.format(CleanFmt) << std::endl;

  // Jの擬似逆行列を計算
  MatrixXd J_pinv = (J.transpose() * J).inverse() * J.transpose();

  // 出力
  std::cout << "J_pinv = \n" << J_pinv.format(CleanFmt) << std::endl;

  // ボディ速度の計算
  Vector6d calc_vel_body;
  calc_vel_body.setZero();
  // 本当はp-pbとなるがpb = 0ならこれでいい
  calc_vel_body.block<3,1>(0, 0) = target_vel_body.block<3,1>(0, 0) - target_vel_body.block<3,1>(3, 0).cross(p_e);
  calc_vel_body.block<3,1>(3, 0) = target_vel_body.block<3,1>(3, 0);

  Vector6d calc_vel_end;
  calc_vel_end.setZero();
  calc_vel_end = target_vel_end - calc_vel_body;

  Vector3d q_dot;
  q_dot.setZero();
  q_dot = J_pinv.cross(calc_vel_end);

  return 0;
}

