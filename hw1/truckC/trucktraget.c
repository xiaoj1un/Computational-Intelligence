#include <stdio.h>
#include <math.h>

#define SPEED 0.1 // 进一步减小步长
#define PI 3.141592653589793
#define MAX_ITERATIONS 10000  // 设置最大迭代次数

typedef struct {
    double x;
    double y;
    double phi;
} Position;

// 模糊控制器的隶属函数
double triangular_mf(double x, double a, double b, double c) {
    if (x <= a || x >= c) return 0.0;
    else if (x == b) return 1.0;
    else if (x < b) return (x - a) / (b - a);
    else return (c - x) / (c - b);
}

// 模糊化输入
double fuzzify_position(double input) {
    if (input < 20) return triangular_mf(input, 0, 10, 20);
    else if (input < 50) return triangular_mf(input, 20, 35, 50);
    else return triangular_mf(input, 50, 65, 80);
}

double fuzzify_angle(double input) {
    if (input < 0) return triangular_mf(input, -80, -40, 0);
    else if (input < 180) return triangular_mf(input, 0, 90, 180);
    else return triangular_mf(input, 180, 220, 260);
}

// 去模糊化
double defuzzify(double position_val, double angle_val) {
    return position_val * 0.6 + angle_val * 0.4;
}

// 初始化位置和角度
Position init_position() {
    Position pos;
    printf("Enter initial position x, y, phi:\n");
    printf("x: ");
    scanf("%lf", &pos.x);
    printf("y: ");
    scanf("%lf", &pos.y);
    printf("phi: ");
    scanf("%lf", &pos.phi);
    return pos;
}

// 计算目标方向角度
double calculate_target_angle(Position current, Position target) {
    double deltaX = target.x - current.x;
    double deltaY = target.y - current.y;
    return atan2(deltaY, deltaX) * 180.0 / PI; // 转换为度数
}

// 更新位置
Position update_position(Position current, double control_val) {
    Position new_pos = current;
    new_pos.phi += control_val;  // 根据控制器输出值调整角度

    // 根据新的角度更新位置
    new_pos.x += SPEED * cos(new_pos.phi * PI / 180.0);
    new_pos.y += SPEED * sin(new_pos.phi * PI / 180.0);
    return new_pos;
}

// 计算 Trajectory Error
double calculate_trajectory_error(double r, int i, double start_x, double start_y) {
    double direct_distance = sqrt(pow(start_x - 50, 2) + pow(start_y - 100, 2));
    return (r * i) / direct_distance;
}

// 主程序
int main() {
    Position current = init_position();
    Position final = {50, 100, 0};  // 终点位置

    double trajectory_length = 0.0;  // 记录轨迹总长度
    Position previous = current;     // 用于存储上一次的位置
    double start_x = current.x;      // 起始位置 x
    double start_y = current.y;      // 起始位置 y
    double r = SPEED;                // 每一步的步长

    // 打开文件用于记录步数和坐标、角度
    FILE *file = fopen("trajectory_data.txt", "w");
    if (!file) {
        printf("Error opening file for writing.\n");
        return 1;
    }

    // 设置更小的误差范围
    double position_tolerance = 0.1;
    double angle_tolerance = 1.0;

    int iteration = 0;

    while ((fabs(current.x - final.x) > position_tolerance || fabs(current.y - final.y) > position_tolerance) && iteration < MAX_ITERATIONS) {
        double target_angle = calculate_target_angle(current, final);
        double angle_error = target_angle - current.phi;

        // 将角度误差控制在 [-180, 180] 范围内
        if (angle_error > 180) angle_error -= 360;
        if (angle_error < -180) angle_error += 360;

        // 动态调整控制增益
        double control_gain = (fabs(angle_error) > 20) ? 1.2 : 0.5;  // 调整控制增益
        double control_val = angle_error * control_gain;

        // 更新位置
        current = update_position(current, control_val);

        // 将步数、x、y 和 phi 写入文件
        fprintf(file, "%d %.5lf %.5lf %.5lf\n", iteration, current.x, current.y, current.phi);

        // 计算并累积轨迹长度
        double step_distance = sqrt(pow(current.x - previous.x, 2) + pow(current.y - previous.y, 2));
        trajectory_length += step_distance;
        
        // 更新上一个位置
        previous = current;
        
        // 输出当前轨迹位置
        printf("Iteration %d: x=%.5lf, y=%.5lf, phi=%.5lf\n", iteration, current.x, current.y, current.phi);
        
        iteration++; // 更新迭代次数
    }

    // 关闭文件
    fclose(file);

    printf("Trajectory data saved to trajectory_data.txt\n");
    system("PAUSE");
    return 0;
}