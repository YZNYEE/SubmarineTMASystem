#include "EKELUND.hpp"
#include <cmath>
#include <numeric>
#include <iostream>
#include <algorithm>

bool Ekelund::estimate(const std::vector<ObsData>& leg1, 
                       const std::vector<ObsData>& leg2,
                       TargetState& result) {
    if (leg1.size() < 2 || leg2.size() < 2) {
        std::cerr << "[Ekelund] Insufficient data points." << std::endl;
        return false;
    }

    // 1. 确定评估时间点 (机动中点)
    double t1_end = leg1.back().timetamp;
    double t2_start = leg2.front().timetamp;
    double t_m = (t1_end + t2_start) / 2.0;

    // 2. 计算两段数据的方位角及变化率 (线性回归)
    // 使用较短的时间窗口来估计机动时刻的瞬时变化率，避免长时段非线性导致的误差
    double bearing1_tm, rate1;
    double bearing2_tm, rate2;
    double calc_window = 120.0; // 使用机动前后 120 秒的数据

    if (!calculateBearingRate(leg1, t_m, calc_window, bearing1_tm, rate1)) return false;
    if (!calculateBearingRate(leg2, t_m, calc_window, bearing2_tm, rate2)) return false;

    // 3. 计算两段数据的平均观测者速度
    double vox1, voy1, vox2, voy2;
    calculateAvgVelocity(leg1, vox1, voy1);
    calculateAvgVelocity(leg2, vox2, voy2);

    // 4. 计算机动时刻的观测者位置 (线性插值/外推)
    // 简单起见，取 leg1 终点和 leg2 起点的中点，或者基于速度外推
    // 这里使用 leg1 终点位置 + 速度 * dt
    double ox1 = leg1.back().x;
    double oy1 = leg1.back().y;
    // 外推到 t_m
    double ox_m = ox1 + vox1 * (t_m - t1_end);
    double oy_m = oy1 + voy1 * (t_m - t1_end);

    // 5. 计算垂直于视线的观测者速度分量
    // 使用平均方位角作为机动时刻的方位角
    double bearing_m = (bearing1_tm + bearing2_tm) / 2.0;
    
    // V_perp = Vx * cos(B) - Vy * sin(B)
    double v_perp1 = vox1 * std::cos(bearing_m) - voy1 * std::sin(bearing_m);
    double v_perp2 = vox2 * std::cos(bearing_m) - voy2 * std::sin(bearing_m);

    // Debug output
    std::cout << "[Ekelund Debug]" << std::endl;
    std::cout << "  Bearing1(tm): " << bearing1_tm << " Rate1: " << rate1 << std::endl;
    std::cout << "  Bearing2(tm): " << bearing2_tm << " Rate2: " << rate2 << std::endl;
    std::cout << "  Bearing(m): " << bearing_m << std::endl;
    std::cout << "  Vox1: " << vox1 << " Voy1: " << voy1 << " Vperp1: " << v_perp1 << std::endl;
    std::cout << "  Vox2: " << vox2 << " Voy2: " << voy2 << " Vperp2: " << v_perp2 << std::endl;
    std::cout << "  Rate Diff: " << rate2 - rate1 << std::endl;
    std::cout << "  Vperp Diff: " << v_perp1 - v_perp2 << std::endl;

    // 6. Ekelund 测距公式
    // R = (V_perp1 - V_perp2) / (rate2 - rate1)
    double rate_diff = rate2 - rate1;
    
    if (std::abs(rate_diff) < 1e-6) {
        std::cerr << "[Ekelund] Bearing rate difference too small (parallel legs or no maneuver)." << std::endl;
        return false;
    }

    double range = (v_perp1 - v_perp2) / rate_diff;

    if (range < 0) {
        std::cerr << "[Ekelund] Negative range calculated (" << range << "). Geometry might be singular." << std::endl;
        // 负距离通常意味着解在反方向，或者数据噪声太大
        return false;
    }

    // 7. 计算目标位置
    result.x = ox_m + range * std::sin(bearing_m);
    result.y = oy_m + range * std::cos(bearing_m);

    // 8. 估算目标速度 (粗略估计)
    // V_t_perp = rate * R + V_o_perp
    // 我们可以用两段的平均值
    double vt_perp = (rate1 * range + v_perp1 + rate2 * range + v_perp2) / 2.0;
    
    // 径向速度无法通过 Ekelund 直接得到，通常假设为 0 或保留未知
    // 如果需要完整速度，需要更多信息。这里仅填充垂直分量贡献
    // 假设径向速度为 0 (最简单的假设)
    double vt_radial = 0.0; 

    // V_tx = Vt_radial * sin(B) + Vt_perp * cos(B)
    // V_ty = Vt_radial * cos(B) - Vt_perp * sin(B)
    result.vx = vt_radial * std::sin(bearing_m) + vt_perp * std::cos(bearing_m);
    result.vy = vt_radial * std::cos(bearing_m) - vt_perp * std::sin(bearing_m);

    return true;
}

bool Ekelund::calculateBearingRate(const std::vector<ObsData>& data, 
                                   double eval_time, 
                                   double window,
                                   double& out_bearing, 
                                   double& out_rate) {
    if (data.size() < 2) return false;

    std::vector<double> times;
    std::vector<double> bearings;

    for (const auto& obs : data) {
        if (std::abs(obs.timetamp - eval_time) <= window) {
            times.push_back(obs.timetamp);
            bearings.push_back(obs.bearing);
        }
    }
    
    if (times.size() < 2) return false;

    unwrapBearings(bearings);

    // 二次回归
    // Bearing(t) = c0 + c1 * t_rel + c2 * t_rel^2
    // t_rel = t - eval_time
    // At eval_time (t_rel = 0), Bearing = c0, Rate = c1

    double s0 = 0, s1 = 0, s2 = 0, s3 = 0, s4 = 0;
    double sy = 0, s1y = 0, s2y = 0;
    int n = times.size();

    for (int i = 0; i < n; ++i) {
        double t = times[i] - eval_time;
        double y = bearings[i];
        
        s0 += 1;
        s1 += t;
        s2 += t * t;
        s3 += t * t * t;
        s4 += t * t * t * t;
        
        sy += y;
        s1y += t * y;
        s2y += t * t * y;
    }

    // Solve 3x3 linear system M * c = R
    // M = [[s0, s1, s2],
    //      [s1, s2, s3],
    //      [s2, s3, s4]]
    // c = [c0, c1, c2]^T
    // R = [sy, s1y, s2y]^T
    
    // Determinant of M
    double det = s0 * (s2 * s4 - s3 * s3) -
                 s1 * (s1 * s4 - s2 * s3) +
                 s2 * (s1 * s3 - s2 * s2);

    if (std::abs(det) < 1e-12) return false;

    // We only need c0 (bearing) and c1 (rate)
    
    // c0 (replace col 0 with R)
    double det0 = sy * (s2 * s4 - s3 * s3) -
                  s1 * (s1y * s4 - s2y * s3) +
                  s2 * (s1y * s3 - s2y * s2);
    
    // c1 (replace col 1 with R)
    // M_c1 = [[s0, sy, s2],
    //         [s1, s1y, s3],
    //         [s2, s2y, s4]]
    double det1 = s0 * (s1y * s4 - s2y * s3) -
                  sy * (s1 * s4 - s2 * s3) +
                  s2 * (s1 * s2y - s2 * s1y);

    out_bearing = det0 / det;
    out_rate = det1 / det;

    while (out_bearing > M_PI) out_bearing -= 2 * M_PI;
    while (out_bearing < -M_PI) out_bearing += 2 * M_PI;

    return true;
}

void Ekelund::calculateAvgVelocity(const std::vector<ObsData>& data, 
                                   double& vx, double& vy) {
    if (data.size() < 2) {
        vx = 0;
        vy = 0;
        return;
    }

    // 假设每段 Leg 内观测者做匀速直线运动
    // 直接利用起点和终点计算平均速度
    const auto& first = data.front();
    const auto& last = data.back();

    double dt = static_cast<double>(last.timetamp - first.timetamp);

    if (dt > 1e-3) {
        vx = (last.x - first.x) / dt;
        vy = (last.y - first.y) / dt;
    } else {
        vx = 0;
        vy = 0;
    }
}

void Ekelund::unwrapBearings(std::vector<double>& bearings) {
    if (bearings.empty()) return;
    for (size_t i = 1; i < bearings.size(); ++i) {
        double diff = bearings[i] - bearings[i-1];
        while (diff > M_PI) {
            bearings[i] -= 2 * M_PI;
            diff = bearings[i] - bearings[i-1];
        }
        while (diff < -M_PI) {
            bearings[i] += 2 * M_PI;
            diff = bearings[i] - bearings[i-1];
        }
    }
}
