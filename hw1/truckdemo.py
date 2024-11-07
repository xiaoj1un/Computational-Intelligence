import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Polygon
from matplotlib.animation import FuncAnimation
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import tkinter as tk
from tkinter import ttk
import threading  # 新增這一行，用於多執行緒
import time  # 新增這一行，用於模擬進度條更新
import concurrent.futures
from concurrent.futures import ThreadPoolExecutor

# 建立 tkinter 視窗
print("開始執行倒車計畫...")
root = tk.Tk()
root.title("模糊控制軌跡模擬")
root.geometry("500x600+600+250")
root.configure(bg="#272727")  

# 初始化變數
fps = 60
r = 50 / fps
frame_num = 1000  # 全域變數
D2R = np.pi / 180
X, Y, phi = None, None, None  # 定義全域變數以儲存每個幀的 X, Y, φ 值
# 停止標誌
stop_flag = False

def set_stop_flag(value):
    global stop_flag
    stop_flag = value

# 定義模糊變數
x_position = ctrl.Antecedent(np.arange(0, 100, 0.1), 'X-position')
angle_phi = ctrl.Antecedent(np.arange(-90, 270), 'Angle')
steering_signal = ctrl.Consequent(np.arange(-30, 30), 'Steering-angle')

# 使用高斯、鐘形和Sigmoid隸屬函數
x_position['LE'] = fuzz.trapmf(x_position.universe, [0, 0,10,35])
x_position['LC'] = fuzz.trimf(x_position.universe, [30, 40, 50])
x_position['CE'] = fuzz.trimf(x_position.universe, [45, 50, 55])
x_position['RC'] = fuzz.trimf(x_position.universe, [50, 60, 70])
x_position['RI'] = fuzz.trapmf(x_position.universe, [65, 90, 100,100])

angle_phi['RB'] = fuzz.trimf(angle_phi.universe, [-100, -50, 0])
angle_phi['RU'] = fuzz.trimf(angle_phi.universe, [-20, 20, 60])
angle_phi['RV'] = fuzz.trimf(angle_phi.universe, [50, 70, 90])
angle_phi['VE'] = fuzz.trimf(angle_phi.universe, [80, 90, 100])
angle_phi['LV'] = fuzz.trimf(angle_phi.universe, [90, 110, 130])
angle_phi['LU'] = fuzz.trimf(angle_phi.universe, [120, 160, 200])
angle_phi['LB'] = fuzz.trimf(angle_phi.universe, [180, 230, 280])

steering_signal['NB'] = fuzz.trimf(steering_signal.universe, [-30,-30,-15])
steering_signal['NM'] = fuzz.trimf(steering_signal.universe, [-25,-15, -5])
steering_signal['NS'] = fuzz.trimf(steering_signal.universe, [-15, -5,  0])
steering_signal['ZE'] = fuzz.trimf(steering_signal.universe, [-10,  0, 10])
steering_signal['PS'] = fuzz.trimf(steering_signal.universe, [  0,  5, 15])
steering_signal['PM'] = fuzz.trimf(steering_signal.universe, [  5, 15, 25])
steering_signal['PB'] = fuzz.trimf(steering_signal.universe, [ 15, 30, 30])


# 添加模糊規則
rules = [
    ctrl.Rule(x_position['LE'] & angle_phi['RB'], steering_signal['PS']),
    ctrl.Rule(x_position['LC'] & angle_phi['RB'], steering_signal['PM']),
    ctrl.Rule(x_position['CE'] & angle_phi['RB'], steering_signal['PM']),
    ctrl.Rule(x_position['RC'] & angle_phi['RB'], steering_signal['PB']),
    ctrl.Rule(x_position['RI'] & angle_phi['RB'], steering_signal['PB']),
    
    ctrl.Rule(x_position['LE'] & angle_phi['RU'], steering_signal['NS']),
    ctrl.Rule(x_position['LC'] & angle_phi['RU'], steering_signal['PS']),
    ctrl.Rule(x_position['CE'] & angle_phi['RU'], steering_signal['PM']),
    ctrl.Rule(x_position['RC'] & angle_phi['RU'], steering_signal['PB']),
    ctrl.Rule(x_position['RI'] & angle_phi['RU'], steering_signal['PB']),
    
    ctrl.Rule(x_position['LE'] & angle_phi['RV'], steering_signal['NM']),
    ctrl.Rule(x_position['LC'] & angle_phi['RV'], steering_signal['NS']),
    ctrl.Rule(x_position['CE'] & angle_phi['RV'], steering_signal['PS']),
    ctrl.Rule(x_position['RC'] & angle_phi['RV'], steering_signal['PM']),
    ctrl.Rule(x_position['RI'] & angle_phi['RV'], steering_signal['PB']),
    
    ctrl.Rule(x_position['LE'] & angle_phi['VE'], steering_signal['NM']),
    ctrl.Rule(x_position['LC'] & angle_phi['VE'], steering_signal['NM']),
    ctrl.Rule(x_position['CE'] & angle_phi['VE'], steering_signal['ZE']),
    ctrl.Rule(x_position['RC'] & angle_phi['VE'], steering_signal['PM']),
    ctrl.Rule(x_position['RI'] & angle_phi['VE'], steering_signal['PM']),
    
    ctrl.Rule(x_position['LE'] & angle_phi['LV'], steering_signal['NB']),
    ctrl.Rule(x_position['LC'] & angle_phi['LV'], steering_signal['NM']),
    ctrl.Rule(x_position['CE'] & angle_phi['LV'], steering_signal['NS']),
    ctrl.Rule(x_position['RC'] & angle_phi['LV'], steering_signal['PS']),
    ctrl.Rule(x_position['RI'] & angle_phi['LV'], steering_signal['PM']),
    
    ctrl.Rule(x_position['LE'] & angle_phi['LU'], steering_signal['NB']),
    ctrl.Rule(x_position['LC'] & angle_phi['LU'], steering_signal['NB']),
    ctrl.Rule(x_position['CE'] & angle_phi['LU'], steering_signal['NM']),
    ctrl.Rule(x_position['RC'] & angle_phi['LU'], steering_signal['NS']),
    ctrl.Rule(x_position['RI'] & angle_phi['LU'], steering_signal['PS']),
    
    ctrl.Rule(x_position['LE'] & angle_phi['LB'], steering_signal['NB']),
    ctrl.Rule(x_position['LC'] & angle_phi['LB'], steering_signal['NB']),
    ctrl.Rule(x_position['CE'] & angle_phi['LB'], steering_signal['NM']),
    ctrl.Rule(x_position['RC'] & angle_phi['LB'], steering_signal['NM']),
    ctrl.Rule(x_position['RI'] & angle_phi['LB'], steering_signal['NS'])
]

# 創建模糊控制系統
steering_ctrl = ctrl.ControlSystem(rules)
steering = ctrl.ControlSystemSimulation(steering_ctrl)

# 車輛座標轉換為三角頂點
def car_coord_convert(car_x, car_y, car_phi):
    car_angle = car_phi * D2R
    triangle_coords = np.array([[car_x, car_y], [car_x - 2.5, car_y - 5], [car_x + 2.5, car_y - 5]])
    rotation_center = triangle_coords[0]
    rotation_matrix = np.array([[np.sin(car_angle), -np.cos(car_angle)],
                                [np.cos(car_angle), np.sin(car_angle)]])
    car_coords = np.dot(triangle_coords - rotation_center, rotation_matrix) + rotation_center
    return car_coords

# 創建新視窗顯示每個幀的 X, Y, φ 值
def show_values_window():
    value_window = tk.Toplevel(root)
    value_window.title("軌跡數據")
    value_window.geometry("520x520+400+400")  # 設定視窗大小

    # 建立顯示每個幀 X, Y, φ 的文本框
    text_area = tk.Text(value_window, height=40, width=60 )
    text_area.pack()

    # 將每個幀的數值添加到文本框
    for i in range(frame_num + 1):
        text_area.insert(tk.END, f"Frame {i}: X={X[i]:.2f}, Y={Y[i]:.2f}, φ={phi[i]:.2f}\n")
    
    # 設置為只讀
    text_area.config(state=tk.DISABLED)

    # 計算並顯示 Docking error 和 Trajectory error
def calculate_errors():
    # 最終位置與目標位置的參數
    x_f, y_f, phi_f = X[frame_num - 1], Y[frame_num - 1], phi[frame_num - 1]
    
    # Docking error 計算
    docking_error = np.sqrt(((phi_f - 90) / 180) ** 2 + ((x_f - 50) / 50) ** 2 + ((y_f - 100) / 100) ** 2)
    
    # Trajectory error 計算
    trajectory_length = np.sum(np.sqrt(np.diff(X[:frame_num])**2 + np.diff(Y[:frame_num])**2))
    initial_to_target_distance = np.sqrt((X[0] - 50) ** 2 + (Y[0] - 100) ** 2)
    trajectory_error = trajectory_length / initial_to_target_distance if initial_to_target_distance != 0 else float('inf')
    
    # 更新主視窗中的顯示標籤
    display_labels["docking_error"].config(text=f"Docking Error: {docking_error:.4f}")
    display_labels["trajectory_error"].config(text=f"Trajectory Error: {trajectory_error:.4f}")

# 執行模擬並計算 Docking error 的函數
def run_simulation(x_init, y_init, phi_init):
    X, Y, phi = np.zeros(frame_num + 1), np.zeros(frame_num + 1), np.zeros(frame_num + 1)
    X[0], Y[0], phi[0] = x_init, y_init, phi_init

    for i in range(frame_num):
        steering.input['X-position'] = X[i]
        steering.input['Angle'] = phi[i]
        steering.compute()

        delta_angle = steering.output['Steering-angle']
        phi[i + 1] = phi[i] + delta_angle
        X[i + 1] = X[i] + r * np.cos(phi[i + 1] * D2R)
        Y[i + 1] = Y[i] + r * np.sin(phi[i + 1] * D2R)

        if abs(Y[i + 1] - 100) < 0.55 and abs(X[i + 1] - 50) < 0.55 and abs(phi[i + 1] - 90) < 5.5:
            break

    # 計算 Docking error
    docking_error = np.sqrt(((phi[i + 1] - 90) / 180) ** 2 + ((X[i + 1] - 50) / 50) ** 2 + ((Y[i + 1] - 100) / 100) ** 2)
    return docking_error

# 動畫和數值顯示啟動函數
def start_simulation():
    global ani, frame_num, X, Y, phi  # 將 frame_num 和數據變數宣告為全域變數
    
    # 讀取用戶輸入的車輛座標
    car_x = float(entry_x.get())
    car_y = float(entry_y.get())
    car_phi = float(entry_phi.get())
    
    # 計算路徑
    X, Y, phi = np.zeros(frame_num + 1), np.zeros(frame_num + 1), np.zeros(frame_num + 1)
    X[0], Y[0], phi[0] = car_x, car_y, car_phi

    for i in range(frame_num):
        steering.input['X-position'] = X[i]
        steering.input['Angle'] = phi[i]
        steering.compute()

        if 'Steering-angle' in steering.output:
            delta_angle = steering.output['Steering-angle']
        else:
            print(f"Error: 'Steering-angle' output not found at frame {i}")
            break

        phi[i + 1] = phi[i] + delta_angle
        X[i + 1] = X[i] + r * np.cos(phi[i + 1] * D2R)
        Y[i + 1] = Y[i] + r * np.sin(phi[i + 1] * D2R)

        if abs(Y[i + 1] - 100) < 0.55 and abs(X[i + 1] - 50) < 0.55 and abs(phi[i + 1] - 90) < 5.5:
            frame_num = i + 1
            print("車輛已成功到達終點")
            calculate_errors()
            break

    # 繪圖設定
    fig, ax = plt.subplots()
    ax.add_patch(Rectangle((0, 0), 100, 100, fc='lightblue'))
    ax.plot(50, 100, color='green', marker='o', markersize=2)
    car_coords = car_coord_convert(car_x, car_y, car_phi)
    car_square = Rectangle((car_x - 2.5, car_y - 2.5), 10, 6, angle=car_phi, facecolor='red')
    ax.add_patch(car_square)
    xdata, ydata = [], []
    car_line, = ax.plot([], [], color='blue', lw=1)

    # 動畫更新函數
    def update(num):
        # 計算左上角的位置，使得右邊邊線的中心點位於 (X[num], Y[num])
        right_x = X[num] - (10 / 2) * np.cos(np.radians(phi[num])) + (6 / 2) * np.sin(np.radians(phi[num]))
        right_y = Y[num] - (10 / 2) * np.sin(np.radians(phi[num])) - (6 / 2) * np.cos(np.radians(phi[num]))

        # 更新車輛矩形的位置和角度
        car_square.set_xy((right_x, right_y))  # 設置左上角位置
        car_square.angle = phi[num]  # 根據當前角度進行旋轉

        # 更新軌跡線
        xdata.append(X[num])
        ydata.append(Y[num])
        car_line.set_data(xdata, ydata)
        
        # 更新顯示的數值
        display_labels["frame"].config(text=f"Frame: {num}")
        display_labels["x"].config(text=f"X: {X[num]:.2f}")
        display_labels["y"].config(text=f"Y: {Y[num]:.2f}")
        display_labels["phi"].config(text=f"φ: {phi[num]:.2f}")
        display_labels["delta_angle"].config(text=f"Steering Angle: {delta_angle:.2f}")
        root.update_idletasks()

        return car_square, car_line
    
    ani = FuncAnimation(fig, update, frames=range(frame_num + 1), repeat=False, interval=1000 / fps)
    

    # 關閉圖形視窗時清除快取並重置輸入與顯示
    def on_close(event):
        if ani.event_source:  # 檢查 event_source 是否存在
            ani.event_source.stop()  # 停止動畫
        plt.close(fig)  # 關閉圖形視窗
        xdata.clear()  # 清空快取數據
        ydata.clear()
        
        # 清空輸入框
        entry_x.delete(0, tk.END)
        entry_y.delete(0, tk.END)
        entry_phi.delete(0, tk.END)
        
        # 清空顯示數值
        display_labels["frame"].config(text="Frame: ")
        display_labels["x"].config(text="X: ")
        display_labels["y"].config(text="Y: ")
        display_labels["phi"].config(text="φ: ")
        display_labels["delta_angle"].config(text="Steering Angle: ")
        
        # 計算並顯示 Docking error 和 Trajectory error
        

    fig.canvas.mpl_connect('close_event', on_close)  # 綁定關閉事件
    plt.axis('scaled')
    plt.show()

# 計算平均 Docking Error 並顯示在 GUI 的函數
def calculate_average_docking_error():
    global stop_flag
    stop_flag = False  # 重置停止標誌
    x_values = [20, 30, 40, 50, 60, 70, 80]
    y_values = [20, 30, 40, 50]
    phi_values = list(range(-80, 261, 5))
    docking_errors = []
    total_iterations = len(x_values) * len(y_values) * len(phi_values)
    current_iteration = [0]  # 使用列表來讓內部函數可以修改這個值

    # 禁用不必要的按鈕
    start_button.config(state=tk.DISABLED)
    show_values_button.config(state=tk.DISABLED)
    calculate_button.config(state=tk.DISABLED)
    stop_button.config(state=tk.NORMAL)  # 啟用「停止」按鈕

    # 初始化進度條和顯示文字
    update_progress_bar(0, total_iterations)
    update_output_text("開始計算...")

    # 定義逐步計算的函數
    def run_single_calculation():
        if stop_flag:  # 檢查是否需要停止
            update_output_text("計算已中止")
            stop_button.config(state=tk.DISABLED)
            start_button.config(state=tk.NORMAL)
            show_values_button.config(state=tk.NORMAL)
            calculate_button.config(state=tk.NORMAL)
            return

        # 獲取當前的 x, y, phi 組合
        x_index = current_iteration[0] // (len(y_values) * len(phi_values))
        y_index = (current_iteration[0] // len(phi_values)) % len(y_values)
        phi_index = current_iteration[0] % len(phi_values)
        x, y, phi = x_values[x_index], y_values[y_index], phi_values[phi_index]

        # 建立新的模糊控制系統實例以避免衝突
        local_steering_ctrl = ctrl.ControlSystem(rules)
        local_steering = ctrl.ControlSystemSimulation(local_steering_ctrl)

        # 執行模擬並計算該組合的 Docking error
        docking_error = run_simulation_and_calculate_error(x, y, phi, local_steering)
        docking_errors.append(docking_error)

        # 更新進度條和終端輸出
        current_iteration[0] += 1
        update_progress_bar(current_iteration[0], total_iterations)
        update_output_text(f"當前進度: {current_iteration[0]}/{total_iterations}, 當前 Docking Error: {docking_error:.8f}")

        # 檢查是否完成所有組合的計算
        if current_iteration[0] < total_iterations:
            # 繼續計算下一個組合
            root.after(10, run_single_calculation)  # 每次延遲10毫秒再呼叫一次
        else:
            # 所有計算完成，計算平均 Docking Error 並顯示結果
            average_docking_error = sum(docking_errors) / len(docking_errors)
            update_output_text(f"計算完成: Average Docking Error = {average_docking_error:.8f}")
            display_labels["average_docking_error"].config(text=f"Average Docking Error: {average_docking_error:.8f}")

            # 恢復按鈕狀態
            stop_button.config(state=tk.DISABLED)
            start_button.config(state=tk.NORMAL)
            show_values_button.config(state=tk.NORMAL)
            calculate_button.config(state=tk.NORMAL)

    # 開始逐步計算
    run_single_calculation()

def run_simulation_and_calculate_error(x, y, phi, steering):
    local_frame_num = 1000
    X, Y, Phi = np.zeros(local_frame_num + 1), np.zeros(local_frame_num + 1), np.zeros(local_frame_num + 1)
    X[0], Y[0], Phi[0] = x, y, phi

    for i in range(local_frame_num):
        steering.input['X-position'] = X[i]
        steering.input['Angle'] = Phi[i]
        steering.compute()

        delta_angle = steering.output['Steering-angle']
        Phi[i + 1] = Phi[i] + delta_angle
        X[i + 1] = X[i] + r * np.cos(Phi[i + 1] * D2R)
        Y[i + 1] = Y[i] + r * np.sin(Phi[i + 1] * D2R)

        if abs(Y[i + 1] - 100) < 0.55 and abs(X[i + 1] - 50) < 0.55 and abs(Phi[i + 1] - 90) < 5.5:
            local_frame_num = i + 1
            break

    x_f, y_f, phi_f = X[local_frame_num - 1], Y[local_frame_num - 1], Phi[local_frame_num - 1]
    docking_error = np.sqrt(((phi_f - 90) / 180) ** 2 + ((x_f - 50) / 50) ** 2 + ((y_f - 100) / 100) ** 2)
    return docking_error

def update_output_text(message):
    output_text.config(state='normal')  # 允許寫入
    output_text.insert(tk.END, message + "\n")  # 在文本框末尾插入新訊息
    output_text.see(tk.END)  # 自動滾動到底部
    output_text.config(state='disabled')  # 禁止編輯
    root.update_idletasks()  # 刷新顯示


# 定義輸入框
input_frame = tk.Frame(root)
input_frame.pack(pady=10)
tk.Label(input_frame, text="X 坐標:", bg="#272727", fg="white").grid(row=0, column=0)
entry_x = tk.Entry(input_frame, bg="#333333", fg="white", relief="flat", highlightthickness=0, insertbackground="white", highlightbackground="#333333")
entry_x.grid(row=0, column=1)
tk.Label(input_frame, text="Y 坐標:", bg="#272727", fg="white").grid(row=1, column=0)
entry_y = tk.Entry(input_frame, bg="#333333", fg="white", relief="flat", highlightthickness=0, insertbackground="white", highlightbackground="#333333")
entry_y.grid(row=1, column=1)
tk.Label(input_frame, text="φ 角度:", bg="#272727", fg="white").grid(row=2, column=0)
entry_phi = tk.Entry(input_frame, bg="#333333", fg="white", relief="flat", highlightthickness=0, insertbackground="white", highlightbackground="#333333")
entry_phi.grid(row=2, column=1)

# 建立並配置啟動按鈕
start_button = tk.Button(root, text="開始模擬", command=start_simulation, bg="#6C6C6C")
start_button.pack()

# 添加顯示數值視窗的按鈕
show_values_button = tk.Button(root, text="顯示軌跡數據", command=show_values_window, bg="#6C6C6C")
show_values_button.pack()

# 添加「計算平均 Docking error」按鈕
calculate_button = tk.Button(root, text="計算平均 Docking Error", command=calculate_average_docking_error, bg="#6C6C6C")
calculate_button.pack()

# 添加終端輸出文本框
output_text = tk.Text(root, height=10, width=60, state='disabled', bg="#000000", fg="white")
output_text.pack(pady=10)

# 添加停止按鈕
stop_button = tk.Button(root, text="停止計算", command=lambda: set_stop_flag(True), bg="#6C6C6C")
stop_button.pack(pady=5)
stop_button.config(state=tk.DISABLED)  # 預設為禁用，只有在計算開始時才啟用
stop_button.config(command=lambda: set_stop_flag(True))

# 定義進度條並添加到界面

style = ttk.Style()
style.theme_use("default")
style.configure("TProgressbar", troughcolor="#333333", background="#2894ff")  # 背景槽顏色和進度顏色
progress_bar = ttk.Progressbar(root, style="TProgressbar", orient="horizontal", length=400, mode="determinate")
progress_bar.pack(pady=10)

# 更新進度條函數
def update_progress_bar(current, total):
    progress_bar["value"] = (current / total) * 100
    root.update()  # 更新界面顯示

# 數值顯示
display_labels = {
    "frame": tk.Label(root, text="Frame: ", bg="#272727", fg="white"),
    "x": tk.Label(root, text="X: ", bg="#272727", fg="white"),
    "y": tk.Label(root, text="Y: ", bg="#272727", fg="white"),
    "phi": tk.Label(root, text="φ: ", bg="#272727", fg="white"),
    "delta_angle": tk.Label(root, text="Steering Angle: ", bg="#272727", fg="white"),
    "docking_error": tk.Label(root, text="Docking Error: ", bg="#272727", fg="white"),
    "trajectory_error": tk.Label(root, text="Trajectory Error: ", bg="#272727", fg="white"),
    "average_docking_error": tk.Label(root, text="平均 Docking Error: ", bg="#272727", fg="white")
}
for label in display_labels.values():
    label.pack()

# 啟動 tkinter 主迴圈
root.mainloop()