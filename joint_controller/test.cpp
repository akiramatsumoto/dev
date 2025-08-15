#include <Eigen/Dense>
#include <iostream>
#include <cmath>
#include <iomanip>

int main() {
  using namespace Eigen;
  using std::cout;
  using std::endl;

  auto rad2deg = [](double r){ return r * 180.0 / M_PI; };

  // ---- Hello ----
//  cout << "Hello, world!" << endl;

  // Joint angles
  double theta_rwl = 0.0; // Right Waist Roll
  double theta_rwp = 0.0; // Right Waist Pitch
  double theta_rkp = 0.0; // Right Knee Pitch

  // Link vectors
  Vector3d vp_c_rwl(-0.091, -0.075,  0.0);
  Vector3d vp_rwl_rwp( 0.091, -0.055, 0.0);
  Vector3d vp_rwp_rkp( 0.0  ,  0.0  , -0.16);
  Vector3d vp_rkp_re ( 0.0  ,  0.0  , -0.173);

  // Target position (from C to RE)
  Vector3d vp_c_re(
    0.0,
    -0.075 + -0.055,
    -0.16 + -0.173
  );
  cout << "vp_c_re: " << vp_c_re.transpose() << endl;


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

  Vector4d fk_result = T_wr * T_wp * T_kp * T_e;
  cout << "fk_result: " << fk_result.transpose() << endl;
  
   // ---------- FK ----------


  return 0;
}

