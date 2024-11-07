import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import matplotlib.pyplot as plt

# 設置卡車的目標位置和目標角度
target_x, target_y, target_phi = 50, 100, 90

# 定義模糊變數（角度 phi 和座標 x, y 作為輸入，方向角 theta 作為輸出）
phi = ctrl.Antecedent(np.arange(-90, 271, 1), 'phi')
x = ctrl.Antecedent(np.arange(0, 101, 1), 'x')
y = ctrl.Antecedent(np.arange(0, 101, 1), 'y')
theta = ctrl.Consequent(np.arange(-30, 31, 1), 'theta')

# 調整隸屬函數
phi['far_right'] = fuzz.trimf(phi.universe, [-90, -45, 0])
phi['right'] = fuzz.trimf(phi.universe, [-45, 0, 90])
phi['center'] = fuzz.trimf(phi.universe, [0, 90, 180])
phi['left'] = fuzz.trimf(phi.universe, [90, 180, 270])
phi['far_left'] = fuzz.trimf(phi.universe, [180, 225, 270])

x['close'] = fuzz.trimf(x.universe, [20, 40, 50])
x['near_target'] = fuzz.trimf(x.universe, [45, 50, 55])  # 接近 50 的範圍
x['far'] = fuzz.trimf(x.universe, [50, 80, 100])

y['below_target'] = fuzz.trimf(y.universe, [0, 50, 100])  # y < 100
y['near_target'] = fuzz.trimf(y.universe, [90, 100, 110])  # 接近 y = 100
y['above_target'] = fuzz.trimf(y.universe, [100, 150, 200])  # y > 100

theta['small_right'] = fuzz.trimf(theta.universe, [5, 10, 15])
theta['large_right'] = fuzz.trimf(theta.universe, [15, 25, 30])
theta['straight'] = fuzz.trimf(theta.universe, [-5, 0, 5])
theta['small_left'] = fuzz.trimf(theta.universe, [-15, -10, -5])
theta['large_left'] = fuzz.trimf(theta.universe, [-30, -25, -15])

# 設置針對 x 接近 50 和 y 接近 100 的模糊規則
rules = [
    # 當 x 接近 50 且 y < 100 時，向上轉動
    ctrl.Rule(phi['center'] & x['near_target'] & y['below_target'], theta['small_right']),
    # 當 x 接近 50 且 y > 100 時，向下轉動
    ctrl.Rule(phi['center'] & x['near_target'] & y['above_target'], theta['small_left']),
    # 當 x 和 y 都接近目標時，保持直行
    ctrl.Rule(phi['center'] & x['near_target'] & y['near_target'], theta['straight']),
    # 其他一些基本的控制規則
    ctrl.Rule(phi['right'] & x['close'] & y['above_target'], theta['small_right']),
    ctrl.Rule(phi['left'] & x['close'] & y['below_target'], theta['small_left']),
    ctrl.Rule(phi['center'] & x['far'] & y['near_target'], theta['straight']),
    ctrl.Rule(phi['right'] & x['close'] & y['near_target'], theta['small_left']),
    ctrl.Rule(phi['left'] & x['close'] & y['near_target'], theta['small_right']),
]

# 初始化控制系統
steering_ctrl = ctrl.ControlSystem(rules)
steering_sim = ctrl.ControlSystemSimulation(steering_ctrl)

# 模擬卡車倒車
def simulate_truck(initial_x, initial_y, initial_phi, max_steps=200):
    r = 1  # 初始移動距離
    trajectory = [(initial_x, initial_y, initial_phi)]
    docking_errors = []

    for step in range(max_steps):
        current_x, current_y, current_phi = trajectory[-1]

        # 設置模糊控制輸入
        steering_sim.input['phi'] = current_phi
        steering_sim.input['x'] = current_x
        steering_sim.input['y'] = current_y
        steering_sim.compute()

        # 檢查 theta 輸出並計算新位置和角度
        theta_adjustment = steering_sim.output.get('theta', 0)  # 默認值為 0
        new_phi = current_phi + theta_adjustment

        # 當卡車接近 x = 50 且 y 接近 100 時減小步長 r
        distance_to_target = np.sqrt((current_x - target_x)**2 + (current_y - target_y)**2)
        r_adjusted = r * (0.1 if distance_to_target < 5 else 0.5 if distance_to_target < 10 else 1)
        new_x = current_x + r_adjusted * np.cos(np.radians(new_phi))
        new_y = current_y + r_adjusted * np.sin(np.radians(new_phi))

        # 計算 Docking error
        docking_error = np.sqrt(((new_phi - target_phi) / 180) ** 2 + 
                                ((new_x - target_x) / 50) ** 2 + 
                                ((new_y - target_y) / 100) ** 2)
        docking_errors.append(docking_error)

        # 更新軌跡
        trajectory.append((new_x, new_y, new_phi))

        # 更嚴格的停止條件
        if abs(new_x - target_x) < 1 and abs(new_y - target_y) < 1 and abs(new_phi - target_phi) < 5:
            print(f"成功抵達目標位置 ({new_x:.2f}, {new_y:.2f}) 和角度 {new_phi:.2f}")
            break

    return trajectory, docking_errors

# 主程式流程
initial_x = float(input("請輸入卡車的初始 X 座標 (0-100): "))
initial_y = float(input("請輸入卡車的初始 Y 座標 (0-100): "))
initial_phi = float(input("請輸入卡車的初始角度 phi (-90到270): "))

# 執行模擬並可視化軌跡
trajectory, docking_errors = simulate_truck(initial_x, initial_y, initial_phi, max_steps=200)
x_values, y_values = zip(*[(pos[0], pos[1]) for pos in trajectory])

# 顯示卡車倒車軌跡
plt.plot(x_values, y_values, marker='o', label='Truck Path')
plt.scatter([target_x], [target_y], color='red', label='Loading Dock (50, 100)')
plt.title('Truck Backing Trajectory')
plt.xlabel('X Position')
plt.ylabel('Y Position')
plt.legend()
plt.show()