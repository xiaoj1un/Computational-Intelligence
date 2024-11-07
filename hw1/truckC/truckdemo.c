#include <stdio.h>
#include <math.h>

#define SPEED 1.0 // 設置初始速度為 1
#define PI 3.141592653589793
#define MAX_ITERATIONS 500  // 最大迭代次數

typedef struct {
    double x;
    double y;
    double phi;
} Position;

// 模糊控制器的隸屬函數
double triangular_mf(double x, double a, double b, double c) {
    if (x <= a || x >= c) return 0.0;
    else if (x == b) return 1.0;
    else if (x < b) return (x - a) / (b - a);
    else return (c - x) / (c - b);
}

// 模糊化輸入 - 根據位置和角度誤差
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
    return position_val * 0.6 + angle_val * 0.4; // 組合控制
}

// 初始化位置和角度
Position init_position() {
    Position pos;
    printf("Enter initial position x, y, phi (phi between -90 and 270):\n");
    printf("x: ");
    scanf("%lf", &pos.x);
    printf("y: ");
    scanf("%lf", &pos.y);
    printf("phi: ");
    scanf("%lf", &pos.phi);

    // 確保 phi 在 -90 到 270 範圍內
    while (pos.phi < -90 || pos.phi > 270) {
        printf("Invalid phi! Enter phi between -90 and 270: ");
        scanf("%lf", &pos.phi);
    }
    return pos;
}

// 計算目標方向角度
double calculate_target_angle(Position current, Position target) {
    double deltaX = target.x - current.x;
    double deltaY = target.y - current.y;
    return atan2(deltaY, deltaX) * 180.0 / PI; // 轉換為度數
}

// 更新位置
Position update_position(Position current, double control_val, double speed) {
    Position new_pos = current;
    new_pos.phi += control_val;

    // 確保 phi 在 -90 到 270 度範圍內
    if (new_pos.phi > 270) new_pos.phi -= 360;
    if (new_pos.phi < -90) new_pos.phi += 360;

    // 根據新的角度更新位置
    new_pos.x += speed * cos(new_pos.phi * PI / 180.0);
    new_pos.y += speed * sin(new_pos.phi * PI / 180.0);
    return new_pos;
}

// 計算 Docking Error
double calculate_docking_error(Position final, Position current) {
    double phi_error = (final.phi - current.phi) / 180.0;
    double x_error = (final.x - current.x) / 50.0;
    double y_error = (final.y - current.y) / 100.0;
    return sqrt(phi_error * phi_error + x_error * x_error + y_error * y_error);
}

// 主程序
int main() {
    Position current = init_position();
    Position final = {50, 100, 90};  // 目標位置

    double trajectory_length = 0.0;  // 紀錄軌跡總長度
    Position previous = current;     // 用於儲存上一次的位置

    // 開啟文件記錄步數和座標、角度
    FILE *file = fopen("trajectory_data.txt", "w");
    if (!file) {
        printf("Error opening file for writing.\n");
        return 1;
    }

    double position_tolerance = 0.5;   // 進一步增加位置容忍範圍

    int iteration = 0;

    while (iteration < MAX_ITERATIONS) {
        // 檢查是否到達目標位置
        if (fabs(current.x - final.x) < position_tolerance &&
            fabs(current.y - final.y) < position_tolerance ) {
            printf("Reached target position and orientation.\n");
            break;
        }

        double target_angle = calculate_target_angle(current, final);
        double angle_error = target_angle - current.phi;

        // 控制角度誤差範圍
        if (angle_error > 180) angle_error -= 360;
        if (angle_error < -180) angle_error += 360;

        // 使用模糊控制器
        double fuzzy_position = fuzzify_position(fabs(final.x - current.x) + fabs(final.y - current.y));
        double fuzzy_angle = fuzzify_angle(angle_error);

        // 增強控制增益：在接近目標時增強控制增益
        double control_gain;
        double distance_to_target = sqrt(pow(final.x - current.x, 2) + pow(final.y - current.y, 2));
        if (distance_to_target < 10) {
            control_gain = 1.2;  // 增加控制增益
        } else {
            control_gain = 0.8;  // 正常增益
        }

        double control_val = defuzzify(fuzzy_position, fuzzy_angle) * angle_error * control_gain;

        // 更新位置
        current = update_position(current, control_val, SPEED);

        // 將步數、x、y 和 phi 寫入文件
        fprintf(file, "%d %.5lf %.5lf %.5lf\n", iteration, current.x, current.y, current.phi);

        // 累積軌跡長度
        double step_distance = sqrt(pow(current.x - previous.x, 2) + pow(current.y - previous.y, 2));
        trajectory_length += step_distance;
        previous = current;

        printf("Iteration %d: x=%.5lf, y=%.5lf, phi=%.5lf\n", iteration, current.x, current.y, current.phi);
        iteration++;
    }

    // 計算 Docking Error
    double docking_error = calculate_docking_error(final, current);
    printf("Docking Error: %.5lf\n", docking_error);

    fclose(file);
    printf("Trajectory data saved to trajectory_data.txt\n");
    system("PAUSE");
    return 0;
}