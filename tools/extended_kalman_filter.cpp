// 【学习导读｜后续参考：通用 EKF 数学实现】
// Target 提供物理模型；此类只负责矩阵计算。x 是状态估计，P 是估计不确定度。
// F：状态转移雅可比；Q：过程噪声；z：观测；h(x)：预计观测；H：观测雅可比；R：观测噪声。
// predict 先推算下一刻，update 再用新测量修正。当前像素跟随没有调用本文件。
// 不必先背矩阵：先理解“按速度预测在哪里，再用本帧检测纠正偏差”。

#include "extended_kalman_filter.hpp"

#include <numeric>

namespace tools
{
ExtendedKalmanFilter::ExtendedKalmanFilter(
  const Eigen::VectorXd & x0, const Eigen::MatrixXd & P0,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> x_add)
: x(x0), P(P0), I(Eigen::MatrixXd::Identity(x0.rows(), x0.rows())), x_add(x_add)
{
  data["residual_yaw"] = 0.0;
  data["residual_pitch"] = 0.0;
  data["residual_distance"] = 0.0;
  data["residual_angle"] = 0.0;
  data["nis"] = 0.0;
  data["nees"] = 0.0;
  data["nis_fail"] = 0.0;
  data["nees_fail"] = 0.0;
  data["recent_nis_failures"] = 0.0;
}

Eigen::VectorXd ExtendedKalmanFilter::predict(const Eigen::MatrixXd & F, const Eigen::MatrixXd & Q)
{
  return predict(F, Q, [&](const Eigen::VectorXd & x) { return F * x; });
}

Eigen::VectorXd ExtendedKalmanFilter::predict(
  const Eigen::MatrixXd & F, const Eigen::MatrixXd & Q,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &)> f)
{
  // 预测协方差 P_prior=F·P·Fᵀ+Q；即使状态预测看起来平滑，不确定度仍需传播。
  // 下一行调用外部传入 f，而非强制 F*x，因此可以处理角度折返等细节。
  P = F * P * F.transpose() + Q;
  x = f(x);
  return x;
}

Eigen::VectorXd ExtendedKalmanFilter::update(
  const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract)
{
  return update(z, H, R, [&](const Eigen::VectorXd & x) { return H * x; }, z_subtract);
}

Eigen::VectorXd ExtendedKalmanFilter::update(
  const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &)> h,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract)
{
  Eigen::VectorXd x_prior = x;
  // K 是卡尔曼增益：把“观测与预测之差”分配回各状态。
  // 在简单情形下，测量噪声 R 越大越少依赖测量，预测不确定度 P 越大越愿接受修正。
  // 实际是矩阵权衡，不能把 K 当成全局固定的 0～1 单个系数。
  Eigen::MatrixXd K = P * H.transpose() * (H * P * H.transpose() + R).inverse();

  // Stable Compution of the Posterior Covariance
  // https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/07-Kalman-Filter-Math.ipynb
  // Joseph 形式更新协方差，比直接写 (I-KH)P 更利于保持数值性质。
  // 并不免除对输入合法性、矩阵病态与浮点误差的检查。
  P = (I - K * H) * P * (I - K * H).transpose() + K * R * K.transpose();

  // z_subtract 处理角度差跨 ±π，x_add 处理更新后状态角度的折返。
  // 函数作为参数传进来，让通用滤波器不必知道哪些下标是角度。
  x = x_add(x, K * z_subtract(z, h(x)));

  /// 卡方检验
  // 【诊断口径待核查】这里已经更新过 x 和 P，下面取的是后验残差。
  // 常规创新一致性检验需使用更新前的创新及其协方差；不能直接按变量名宣称等价。
  // 下面 nees 用的是 x-x_prior（更新量），并没有真实状态误差这个输入。
  Eigen::VectorXd residual = z_subtract(z, h(x));
  // 新增检验
  Eigen::MatrixXd S = H * P * H.transpose() + R;
  double nis = residual.transpose() * S.inverse() * residual;
  double nees = (x - x_prior).transpose() * P.inverse() * (x - x_prior);

  // 卡方检验阈值（自由度=4，取置信水平95%）
  // 0.711 是现存代码阈值；上方原“自由度4/95%”注释不能当作校验依据。
  // 接入前应核查统计量定义与阈值来源，当前只把它当作原实现的经验诊断门槛。
  constexpr double nis_threshold = 0.711;
  constexpr double nees_threshold = 0.711;

  if (nis > nis_threshold) nis_count_++, data["nis_fail"] = 1;
  if (nees > nees_threshold) nees_count_++, data["nees_fail"] = 1;
  total_count_++;
  last_nis = nis;

  // 保存最近窗口的超阈值标记，Tracker 根据失败次数决定是否重置目标。
  // 这说明诊断量也会影响跟踪行为，不能只当成不影响结果的打印信息。
  recent_nis_failures.push_back(nis > nis_threshold ? 1 : 0);

  if (recent_nis_failures.size() > window_size) {
    recent_nis_failures.pop_front();
  }

  int recent_failures = std::accumulate(recent_nis_failures.begin(), recent_nis_failures.end(), 0);
  double recent_rate = static_cast<double>(recent_failures) / recent_nis_failures.size();

  data["residual_yaw"] = residual[0];
  data["residual_pitch"] = residual[1];
  data["residual_distance"] = residual[2];
  data["residual_angle"] = residual[3];
  data["nis"] = nis;
  data["nees"] = nees;
  data["recent_nis_failures"] = recent_rate;

  return x;
}

}  // namespace tools