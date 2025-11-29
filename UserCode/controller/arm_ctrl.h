#pragma once

#include "interfaces/arm_motor_if.h"

namespace Arm
{

    /**
     * @brief 机械臂控制器 (2-DOF + Gripper)
     */
    class Controller
    {
    public:
        /**
         * @brief 机械臂物理参数配置
         */
        struct Config
        {
            // 连杆长度 (m)
            float l1; // 大臂长度
            float l2; // 小臂长度

            // 连杆质心位置 (距离关节轴的距离, m)
            float lc1;
            float lc2;

            // 连杆质量 (kg)
            float m1;
            float m2;
            float m_gripper; // 夹爪质量 (视为小臂末端的点质量)

            // 重力加速度
            float g;
        };

        /**
         * @brief 构造函数
         * @param joint1 大臂电机 (Unitree)
         * @param joint2 小臂电机 (DJI 3508)
         * @param gripper 夹爪电机 (DJI 2006)
         * @param config 机械臂物理参数
         */
        Controller(Motor& joint1, Motor& joint2, Motor& gripper, const Config& config);

        /**
         * @brief 初始化控制器
         */
        void init();

        /**
         * @brief 设置关节目标角度 (关节空间控制)
         * @param q1 大臂目标角度 (degree)
         * @param q2 小臂目标角度 (degree)
         * @param t1 大臂运动时间 (s)
         * @param t2 小臂运动时间 (s)
         */
        void setJointTarget(float q1, float q2, float t1, float t2);

        /**
         * @brief 查询关节是否都已到达目标
         * @return true 已到达, false 运动中
         */
        bool isArrived() const;

        /**
         * @brief 控制夹爪开合
         * @param open_width 开合宽度或角度 (根据具体夹爪实现定义)
         */
        void setGripper(float open_width);

        /**
         * @brief 更新控制回路 (需周期性调用)
         * @param dt 距离上一次调用的时间间隔 (s)
         * @note 此函数会计算重力补偿并更新电机
         */
        void update(float dt);

        /**
         * @brief 正运动学求解 (FK)
         * @param x [out] 末端 X 坐标 (m)
         * @param y [out] 末端 Y 坐标 (m)
         */
        void getEndEffectorPose(float& x, float& y) const;

        /**
         * @brief 获取当前关节角度
         */
        void getJointAngles(float& q1, float& q2) const;

    private:
        /**
         * @brief 五次多项式轨迹规划器
         */
        struct QuinticTrajectory
        {
            float c0, c1, c2, c3, c4, c5; // 多项式系数
            float current_time;           // 当前运行时间
            float total_time;             // 总运行时间
            float target_pos;             // 最终目标位置 (用于修正浮点误差)
            bool running;                 // 是否正在运行

            QuinticTrajectory() :
                running(false) {}

            /**
             * @brief 规划轨迹
             * @param start_pos 起始位置
             * @param start_vel 起始速度
             * @param end_pos 结束位置
             * @param time 运行时间
             */
            void plan(float start_pos, float start_vel, float end_pos, float time);

            /**
             * @brief 计算当前时刻的位置和速度
             * @param dt 时间增量
             * @param pos [out] 位置
             * @param vel [out] 速度
             */
            void step(float dt, float& pos, float& vel);
        };

        /**
         * @brief 计算重力补偿力矩
         * @param q1 当前大臂角度
         * @param q2 当前小臂角度
         * @param tau1 [out] 大臂补偿力矩
         * @param tau2 [out] 小臂补偿力矩
         */
        void calculateGravityComp(float q1, float q2, float& tau1, float& tau2);

        Motor& joint1_;  // 大臂
        Motor& joint2_;  // 小臂
        Motor& gripper_; // 夹爪

        Config config_;

        QuinticTrajectory traj_q1_;
        QuinticTrajectory traj_q2_;

        float current_q1_ref_; // 当前规划的位置参考值
        float current_q2_ref_;
        float target_gripper_;
    };

} // namespace Arm
