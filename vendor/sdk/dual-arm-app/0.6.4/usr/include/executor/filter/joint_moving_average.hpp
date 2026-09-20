#include <iostream>
#include <vector>
class JointMovingAverage
{
public:
    int moving_step = 10;
    int count = 0;
    std::vector<int> current_index;
    std::vector<std::vector<double>> data;
    void init()
    {
        data.resize(14);
        current_index.resize(14);
        for (int i = 0; i < data.size(); i++)
        {
            data[i].resize(moving_step);
            current_index[i] = 0;
        }
    }
    double limit_angle(int i, double target_angle)
    {
        if (count < moving_step * 15)
        {
            count++;
        }
        data[i][current_index[i]] = target_angle;
        current_index[i] = (current_index[i] + 1) % moving_step;
        if (count < moving_step * 14)
        {
            return target_angle;
        }
        double sum = 0;
        for (int j = 0; j < moving_step; j++)
        {
            sum = sum + data[i][j];
            if (i == 3)
            {
                std::cout << "j: " << j << "num: " << data[i][j] << "sum: " << sum << std::endl;
            }
        }
        return sum / moving_step;
    }
};