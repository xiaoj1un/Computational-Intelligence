import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

# 读取数据
iterations = []
xs = []
ys = []
phis = []

with open("trajectory_data.txt", "r") as file:
    for line_num, line in enumerate(file, start=1):
        data = line.split()
        if len(data) < 4:
            print(f"Warning: Line {line_num} is malformed: {line.strip()}")
            continue  # 跳过格式不正确的行
        iterations.append(int(data[0]))
        xs.append(float(data[1]))
        ys.append(float(data[2]))
        phis.append(float(data[3]))

# 创建图形
fig, ax = plt.subplots(figsize=(8, 8))
ax.fill_between([0, 100], 0, 100, color='skyblue', alpha=0.4)  # 设置背景区域
ax.set_xlim(0, 100)
ax.set_ylim(0, 100)
ax.set_xlabel("X Position")
ax.set_ylabel("Y Position")
ax.set_title("Vehicle Trajectory with Direction Arrows")
ax.grid()

# 绘制起点（绿色）和终点（红色）
ax.scatter([xs[0]], [ys[0]], color='green', s=100, label='Start')  # 起点
ax.scatter([xs[-1]], [ys[-1]], color='red', marker='^', s=100, label='End')  # 终点
ax.legend()

# 初始化线条和箭头
trajectory_line, = ax.plot([], [], color='b', linewidth=2, label='Trajectory')
arrow = None  # 初始箭头为 None

# 更新每一帧
def update(frame):
    global arrow
    # 更新轨迹线
    trajectory_line.set_data(xs[:frame], ys[:frame])
    
    # 更新箭头方向
    if arrow:
        arrow.remove()  # 如果箭头已存在，移除它
    if frame < len(xs):
        dx = np.cos(phis[frame] * np.pi / 180) * 2
        dy = np.sin(phis[frame] * np.pi / 180) * 2
        arrow = ax.arrow(xs[frame], ys[frame], dx, dy, head_width=1, head_length=2, fc='blue', ec='blue')

    return trajectory_line, arrow

# 创建动画
ani = animation.FuncAnimation(fig, update, frames=len(xs), interval=30, blit=True)

# 显示动画
plt.show()