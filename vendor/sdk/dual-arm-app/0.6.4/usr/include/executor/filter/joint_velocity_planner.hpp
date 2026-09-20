// #include <Eigen/src/Core/Matrix.h>
#include <iostream>
#include <plog/Log.h>
#include <vector>
class JointVelocityPlanner
{
public:
    // std::vector<double> max_velocity = {1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5}; // 最大允许关节速度
    std::vector<double> max_velocity = {1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5}; // 最大允许关节速度
    // std::vector<double> max_acc = {6.5, 6.5, 6.5, 6.5, 6.5, 6.5, 6.5, 5, 5, 5, 5, 5, 5, 5};      // 最大允许关节加速度
    std::vector<double> max_acc = {6.5, 6.5, 6.5, 6.5, 6.5, 6.5, 6.5};      // 最大允许关节加速度
    // std::vector<double> max_velocity = {9999, 9999, 9999, 9999, 9999, 9999, 9999, 9999, 9999, 9999, 9999, 9999, 9999, 9999}; // 最大允许关节速度
    // std::vector<double> max_acc = {9999, 9999, 9999, 9999, 9999, 9999, 9999, 9999, 9999, 9999, 9999, 9999, 9999, 9999};      // 最大允许关节加速度
    std::vector<double> last_angle = {0, 0, 0, 0, 0, 0, 0};    // 上一时刻的关节角度
    std::vector<double> last_velocity = {0, 0, 0, 0, 0, 0, 0}; // 上一时刻的关节速度

    double tolerance = 0.1;
public:
    // 计算受限后的关节角度
    double limit_angle(int i, double target_angle, double dt)
    {

        // 计算目标速度
        double target_velocity = (target_angle - last_angle[i]) / dt;

        // 计算最大允许的速度变化
        double max_velocity_change = max_acc[i] * dt;

        // 限制速度变化
        double new_velocity = last_velocity[i];
        if (target_velocity > last_velocity[i] + max_velocity_change)
        {
            new_velocity = last_velocity[i] + max_velocity_change;
        }
        else if (target_velocity < last_velocity[i] - max_velocity_change)
        {
            new_velocity = last_velocity[i] - max_velocity_change;
        }
        else
        {
            new_velocity = target_velocity;
        }

        // 限制速度
        if (new_velocity > max_velocity[i])
        {
            new_velocity = max_velocity[i];
        }
        else if (new_velocity < -max_velocity[i])
        {
            new_velocity = -max_velocity[i];
        }
        

        // 如果接近目标角度，逐渐减小速度
        // if (std::abs(last_angle[i] - target_angle) < last_velocity[i] /15)
        if (std::abs(last_angle[i] - target_angle) < std::abs(last_velocity[i] )/10)
        {
            // PLOGI<<"进入关节"<<i<<"修正速度，角度差"<<std::abs(last_angle[i] - target_angle)<<"上一速度"<<last_velocity[i];
            if (new_velocity > 0)
            {
                new_velocity = std::max(0.0, new_velocity - 2 * 6.5 * dt);
            }
            else
            {
                new_velocity = std::min(0.0, new_velocity + 2 * 6.5 * dt);
            }
        }

        // if()
        // double temp_a=(target_velocity-new_velocity)/dt;
        // if(temp_a>15*max_acc[i]){
        //     // if(new_velocity<0){
        //         new_velocity = new_velocity+max_acc[i]*dt;
        //     // }
        // }
        // else if(temp_a<-15*max_acc[i]){
        //     // if(new_velocity>0){
        //         new_velocity=new_velocity-max_acc[i]*dt;
        //     // }
        // }

        // 限制速度
        // if (new_velocity > max_velocity[i])
        // {
        //     new_velocity = max_velocity[i];
        // }
        // else if (new_velocity < -max_velocity[i])
        // {
        //     new_velocity = -max_velocity[i];
        // }


        // 计算新的关节角度
        double new_angle = last_angle[i] + new_velocity * dt;
        // PLOGI<<"vel_acc:"<<"关节"<<i<<"速度:"<<new_velocity<<"加速度:"<<(new_velocity-last_velocity[i])/dt;
        // 更新状态
        last_angle[i] = new_angle;
        last_velocity[i] = new_velocity;


        return new_angle;
    }

};