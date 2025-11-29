#include "arm_ctrl.h"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static constexpr float RAD_TO_DEG = 180.0f / M_PI;
static constexpr float DEG_TO_RAD = M_PI / 180.0f;

namespace Arm
{

    // ============================================================================
    // QuinticTrajectory Implementation
    // ============================================================================

    void Controller::QuinticTrajectory::plan(float start_pos, float start_vel, float end_pos, float time)
    {
        if (time <= 0.0001f)
        {
            // 时间太短，直接到达
            c0           = end_pos;
            c1           = 0;
            c2           = 0;
            c3           = 0;
            c4           = 0;
            c5           = 0;
            current_time = 0;
            total_time   = 0;
            target_pos   = end_pos;
            running      = false;
            return;
        }

        total_time   = time;
        current_time = 0;
        target_pos   = end_pos;
        running      = true;

        // 计算五次多项式系数
        // 约束: p0=start, v0=vel, a0=0; pf=end, vf=0, af=0
        float h  = end_pos - start_pos;
        float T  = time;
        float T2 = T * T;
        float T3 = T2 * T;
        float T4 = T3 * T;
        float T5 = T4 * T;

        c0 = start_pos;
        c1 = start_vel;
        c2 = 0.0f; // 假设初始加速度为 0，如果需要更平滑的连续运动，需要记录上一次的加速度

        // c3 = (10h - 6v0T) / T^3
        c3 = (10.0f * h - 6.0f * start_vel * T) / T3;

        // c4 = (15(p0 - pf) + 8v0T) / T^4  => (-15h + 8v0T) / T^4
        c4 = (-15.0f * h + 8.0f * start_vel * T) / T4;

        // c5 = (6h - 3v0T) / T^5
        c5 = (6.0f * h - 3.0f * start_vel * T) / T5;
    }

    void Controller::QuinticTrajectory::step(float dt, float& pos, float& vel)
    {
        if (!running)
        {
            pos = target_pos;
            vel = 0.0f;
            return;
        }

        current_time += dt;

        if (current_time >= total_time)
        {
            pos     = target_pos;
            vel     = 0.0f;
            running = false;
            return;
        }

        float t  = current_time;
        float t2 = t * t;
        float t3 = t2 * t;
        float t4 = t3 * t;
        float t5 = t4 * t;

        // p(t) = c0 + c1*t + c2*t^2 + c3*t^3 + c4*t^4 + c5*t^5
        pos = c0 + c1 * t + c2 * t2 + c3 * t3 + c4 * t4 + c5 * t5;

        // v(t) = c1 + 2*c2*t + 3*c3*t^2 + 4*c4*t^3 + 5*c5*t^4
        vel = c1 + 2.0f * c2 * t + 3.0f * c3 * t2 + 4.0f * c4 * t3 + 5.0f * c5 * t4;
    }

    // ============================================================================
    // Controller Implementation
    // ============================================================================

    Controller::Controller(Motor& joint1, Motor& joint2, Motor& gripper, const Config& config) :
        joint1_(joint1), joint2_(joint2), gripper_(gripper), config_(config),
        current_q1_ref_(0.0f), current_q2_ref_(0.0f), target_gripper_(0.0f)
    {
    }

    void Controller::init()
    {
        // 读取初始位置作为目标位置，防止上电跳变
        // 注意: getAngle 返回弧度，这里转换为度存储在内部状态中
        float q1_deg = joint1_.getAngle() * RAD_TO_DEG;
        float q2_deg = joint2_.getAngle() * RAD_TO_DEG;

        current_q1_ref_ = q1_deg;
        current_q2_ref_ = q2_deg;
        target_gripper_ = gripper_.getAngle() * RAD_TO_DEG; // 假设 gripper 也是弧度接口

        // 初始化轨迹为当前位置，速度为0
        traj_q1_.plan(q1_deg, 0, q1_deg, 0);
        traj_q2_.plan(q2_deg, 0, q2_deg, 0);
    }

    void Controller::setJointTarget(float q1, float q2, float t1, float t2)
    {
        // 获取当前状态作为起点 (解决速度跳变问题)
        // 使用当前规划器的输出作为起点可能比使用传感器反馈更平滑，
        // 但如果偏差较大，使用反馈更安全。这里为了连续性，使用当前电机的实际反馈速度。
        // 位置则使用当前规划的参考位置，避免因控制误差导致的轨迹回跳。

        float start_q1 = current_q1_ref_;
        float start_v1 = joint1_.getVelocity() * RAD_TO_DEG;

        float start_q2 = current_q2_ref_;
        float start_v2 = joint2_.getVelocity() * RAD_TO_DEG;

        traj_q1_.plan(start_q1, start_v1, q1, t1);
        traj_q2_.plan(start_q2, start_v2, q2, t2);
    }

    bool Controller::isArrived() const
    {
        return !traj_q1_.running && !traj_q2_.running;
    }

    void Controller::setGripper(float open_width)
    {
        target_gripper_ = open_width;
    }

    void Controller::update(float dt)
    {
        // 1. 计算轨迹生成的新目标状态
        float q1_vel_ref, q2_vel_ref;

        traj_q1_.step(dt, current_q1_ref_, q1_vel_ref);
        traj_q2_.step(dt, current_q2_ref_, q2_vel_ref);

        // 2. 计算重力补偿力矩 (需要弧度)
        float q1_rad = current_q1_ref_ * DEG_TO_RAD;
        float q2_rad = current_q2_ref_ * DEG_TO_RAD;

        float tau1_g = 0.0f;
        float tau2_g = 0.0f;
        calculateGravityComp(q1_rad, q2_rad, tau1_g, tau2_g);

        // 3. 更新关节电机目标 (位置 + 前馈力矩)
        // setTarget 接受度数
        joint1_.setTarget(current_q1_ref_, tau1_g);
        joint2_.setTarget(current_q2_ref_, tau2_g);

        // 4. 更新夹爪电机
        gripper_.setTarget(target_gripper_, 0.0f);

        // 5. 执行底层控制更新
        joint1_.update();
        joint2_.update();
        gripper_.update();
    }

    void Controller::getEndEffectorPose(float& x, float& y) const
    {
        float q1 = joint1_.getAngle(); // 弧度
        float q2 = joint2_.getAngle(); // 弧度

        float l1 = config_.l1;
        float l2 = config_.l2;

        x = l1 * cosf(q1) + l2 * cosf(q1 + q2);
        y = l1 * sinf(q1) + l2 * sinf(q1 + q2);
    }

    void Controller::getJointAngles(float& q1, float& q2) const
    {
        q1 = joint1_.getAngle();
        q2 = joint2_.getAngle();
    }

    void Controller::calculateGravityComp(float q1, float q2, float& tau1, float& tau2)
    {
        // 动力学参数
        float m1  = config_.m1;
        float m2  = config_.m2;
        float mg  = config_.m_gripper;
        float l1  = config_.l1;
        float lc1 = config_.lc1;
        float lc2 = config_.lc2;
        float g   = config_.g;

        // 关节 2 (小臂) 的重力矩
        // 负载包括: 小臂自重 + 夹爪(末端负载)
        // T2 = m2 * g * lc2 * cos(q1 + q2) + mg * g * l2 * cos(q1 + q2)
        // 注意: 角度定义假设水平向右为 0 度，逆时针为正
        float cos_q12 = cosf(q1 + q2);
        tau2          = (m2 * lc2 + mg * config_.l2) * g * cos_q12;

        // 关节 1 (大臂) 的重力矩
        // 负载包括: 大臂自重 + 小臂自重 + 夹爪
        // T1 = m1 * g * lc1 * cos(q1) + T2_projected_to_joint1
        // 更精确的拉格朗日动力学推导:
        // T1 = (m1 * lc1 + m2 * l1 + mg * l1) * g * cos(q1) + (m2 * lc2 + mg * l2) * g * cos(q1 + q2)

        float cos_q1 = cosf(q1);
        tau1         = (m1 * lc1 + m2 * l1 + mg * l1) * g * cos_q1 + tau2;
    }

} // namespace Arm
