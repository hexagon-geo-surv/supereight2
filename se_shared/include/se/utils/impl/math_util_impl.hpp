#ifndef SE_MATH_UTIL_IMPL_HPP
#define SE_MATH_UTIL_IMPL_HPP

namespace se {
namespace math {



template <typename T>
constexpr bool is_power_of_two(T) { return false; }



template<>
constexpr bool is_power_of_two<unsigned int>(const unsigned int x) { return (x != 0) && ((x & (x - 1)) == 0); }



template<>
constexpr bool is_power_of_two<int>(const int x) { return (x != 0) && ((x & (x - 1)) == 0); }



constexpr int log2_const(int n){
  return (n < 2 ? 0 : 1 + log2_const(n/2));
}



static inline unsigned power_two_up(const float x) {
  int p = ceil(log(x)/log(2));
  return (1 << p);
}



template <typename T>
static inline T fracf(const T& v) {
  return v - v.array().floor().matrix();
}



template <typename T>
static inline T floorf(const T& v) {
  return v.array().floor();
}



template <typename T>
static inline T fabs(const T& v) {
  return v.cwiseAbs();
}



template <typename Scalar>
static constexpr inline Scalar sq(Scalar a) {
  return a * a;
}



template <typename Scalar>
static constexpr inline Scalar cu(Scalar a) {
  return a * a * a;
}



template <typename Scalar>
static inline bool in(const Scalar v,
                      const Scalar a,
                      const Scalar b) {
  return v >= a && v <= b;
}



static inline Eigen::Vector3f to_translation(const Eigen::Matrix4f& T) {
  Eigen::Vector3f t = T.block<3,1>(0,3);
  return t;
}



static inline Eigen::Matrix3f to_rotation(const Eigen::Matrix4f& T) {
  Eigen::Matrix3f R = T.block<3,3>(0,0);
  return R;
}



static inline Eigen::Matrix4f to_transformation(const Eigen::Vector3f& t) {
  Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
  T.block<3,1>(0,3) = t;
  return T;
}



static inline Eigen::Matrix4f to_transformation(const Eigen::Matrix3f& R, const Eigen::Vector3f& t) {
  Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
  T.block<3,3>(0,0) = R;
  T.block<3,1>(0,3) = t;
  return T;
}



static inline Eigen::Vector3f to_inverse_translation(const Eigen::Matrix4f& T) {
  Eigen::Vector3f t_inv = -T.block<3,3>(0,0).inverse() * T.block<3,1>(0,3);
  return t_inv;
}



static inline Eigen::Matrix3f to_inverse_rotation(const Eigen::Matrix4f& T) {
  Eigen::Matrix3f R_inv = (T.block<3,3>(0,0)).inverse();
  return R_inv;
}



static inline Eigen::Matrix4f to_inverse_transformation(const Eigen::Matrix4f& T) {
  Eigen::Matrix4f T_inv = Eigen::Matrix4f::Identity();
  T_inv.block<3,3>(0,0) = T.block<3,3>(0,0).inverse();
  T_inv.block<3,1>(0,3) = -T.block<3,3>(0,0).inverse() * T.block<3,1>(0,3);
  return T_inv;
}



template <typename T>
static inline typename std::enable_if<std::is_arithmetic<T>::value, T>::type
clamp(const T& f, const T& a, const T& b) {
  return std::max(a, std::min(f, b));
}



static inline void clamp(Eigen::Ref<Eigen::VectorXf>             res,
                         const Eigen::Ref<const Eigen::VectorXf> a,
                         const Eigen::Ref<Eigen::VectorXf>       b) {
  res = (res.array() < a.array()).select(a, res);
  res = (res.array() >= b.array()).select(b, res);
}



template <typename R, typename A, typename B>
static inline void clamp(Eigen::MatrixBase<R>&       res,
                         const Eigen::MatrixBase<A>& a,
                         const Eigen::MatrixBase<B>& b) {
  res = res.array().max(a.array());
  res = res.array().min(b.array());
}



static Eigen::Vector4f plane_normal(const Eigen::Vector4f& p1,
                                    const Eigen::Vector4f& p2,
                                    const Eigen::Vector4f& p3) {
  // Plane tangent vectors
  const Eigen::Vector3f t1 = p2.head<3>() - p1.head<3>();
  const Eigen::Vector3f t2 = p3.head<3>() - p2.head<3>();
  // Unit normal vector
  return t1.cross(t2).normalized().homogeneous();
}



template <typename T>
static T median(std::vector<T>& data) {
  if (!data.empty()) {
    std::sort(data.begin(), data.end());
    // Compute both the quotient and remainder in one go
    const std::ldiv_t result = std::ldiv(data.size(), 2);
    const size_t mid_idx = result.quot;
    if (result.rem == 0) {
      return (data[mid_idx - 1] + data[mid_idx]) / 2;
    } else {
      return data[mid_idx];
    }
  } else {
    return T();
  }
}



template <typename T>
static T almost_median(std::vector<T>& data) {
  if (!data.empty()) {
    std::sort(data.begin(), data.end());
    const size_t mid_idx = data.size() / 2;
    return data[mid_idx];
  } else {
    return T();
  }
}



template <typename T>
static T median(const std::vector<T>& data) {
  std::vector<T> v (data);
  return median(v);
}



static Eigen::Matrix3f hat(const Eigen::Vector3f& omega) {
  Eigen::Matrix3f Omega;
  // clang-format off
  Omega <<       0.f, -omega(2),  omega(1),
          omega(2),       0.f, -omega(0),
          -omega(1),  omega(0),       0.f;
  // clang-format on
  return Omega;
}



static Eigen::Matrix3f exp_and_theta(const Eigen::Vector3f& omega,
                                     float&                 theta) {
  using std::sqrt;
  using std::abs;
  using std::sin;
  using std::cos;

  float theta_sq = omega.squaredNorm();
  theta = std::sqrt(theta_sq);
  float half_theta = 0.5f * theta;

  float imag_factor;
  float real_factor;
  if (theta < 1e-10) {
    float theta_po4 = theta_sq * theta_sq;
    imag_factor = 0.5f - 1.0f / 48.0f * theta_sq +
                  1.0f / 3840.0f * theta_po4;
    real_factor = 1.f - 0.5f * theta_sq + 1.f / 384.f * theta_po4;
  } else {
    float sin_half_theta = sin(half_theta);
    imag_factor = sin_half_theta / theta;
    real_factor = cos(half_theta);
  }

  Eigen::Quaternionf q =
          Eigen::Quaternionf(real_factor, imag_factor * omega.x(),
                             imag_factor * omega.y(), imag_factor * omega.z());
  return q.normalized().toRotationMatrix();
}



static Eigen::Matrix4f exp(const Eigen::Matrix<float, 6, 1>& a) {
  using std::cos;
  using std::sin;
  const Eigen::Vector3f omega = a.tail<3>();

  float theta;
  const Eigen::Matrix3f so3 = se::math::exp_and_theta(omega, theta);
  const Eigen::Matrix3f Omega = se::math::hat(omega);
  const Eigen::Matrix3f Omega_sq = Omega * Omega;
  Eigen::Matrix3f V;

  if (theta < 1e-10) {
    V = so3;
    /// Note: That is an accurate expansion!
  } else {
    float theta_sq = theta * theta;
    V = (Eigen::Matrix3f::Identity() +
         (1.f - cos(theta)) / (theta_sq) * Omega +
         (theta - sin(theta)) / (theta_sq * theta) * Omega_sq);
  }

  Eigen::Matrix4f se3 = Eigen::Matrix4f::Identity();
  se3.block<3,3>(0,0) = so3;
  se3.block<3,1>(0,3) = V * a.head<3>();
  return se3;
}

} // namespace math
} // namespace se

#endif // SE_MATH_UTIL_IMPL_HPP
