import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Polygon
from matplotlib.animation import FuncAnimation
import skfuzzy as fuzz
from skfuzzy import control as ctrl
###############獲取用戶輸入的車輛座標###############
car_x = float(input("請輸入車輛的X坐標:"))
car_y = float(input("請輸入車輛的Y坐標:"))
car_φ = float(input("請輸入車輛的角度:"))
fps=60 #幀數
r=50 /fps#速度
frame_num=1000#幀數上限
###################################################
# 定義模糊變數
x_position = ctrl.Antecedent(np.arange(0, 100, 0.1), label='X-position')
angle_phi = ctrl.Antecedent(np.arange(-90, 270), label='Angle')
steering_signal = ctrl.Consequent(np.arange(-30, 30), label='Steering-angle')

# 定義模糊集合
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

# 定義模糊規則
rule1 = ctrl.Rule(x_position['LE'] & angle_phi['RB'], steering_signal['PS'])
rule2 = ctrl.Rule(x_position['LC'] & angle_phi['RB'], steering_signal['PM'])
rule3 = ctrl.Rule(x_position['CE'] & angle_phi['RB'], steering_signal['PM'])
rule4 = ctrl.Rule(x_position['RC'] & angle_phi['RB'], steering_signal['PB'])
rule5 = ctrl.Rule(x_position['RI'] & angle_phi['RB'], steering_signal['PB'])

rule6 = ctrl.Rule(x_position['LE'] & angle_phi['RU'], steering_signal['NS'])
rule7 = ctrl.Rule(x_position['LC'] & angle_phi['RU'], steering_signal['PS'])
rule8 = ctrl.Rule(x_position['CE'] & angle_phi['RU'], steering_signal['PM'])
rule9 = ctrl.Rule(x_position['RC'] & angle_phi['RU'], steering_signal['PB'])
rule10 = ctrl.Rule(x_position['RI'] & angle_phi['RU'], steering_signal['PB'])

rule11 = ctrl.Rule(x_position['LE'] & angle_phi['RV'], steering_signal['NM'])
rule12 = ctrl.Rule(x_position['LC'] & angle_phi['RV'], steering_signal['NS'])
rule13 = ctrl.Rule(x_position['CE'] & angle_phi['RV'], steering_signal['PS'])
rule14 = ctrl.Rule(x_position['RC'] & angle_phi['RV'], steering_signal['PM'])
rule15 = ctrl.Rule(x_position['RI'] & angle_phi['RV'], steering_signal['PB'])

rule16 = ctrl.Rule(x_position['LE'] & angle_phi['VE'], steering_signal['NM'])
rule17 = ctrl.Rule(x_position['LC'] & angle_phi['VE'], steering_signal['NM'])
rule18 = ctrl.Rule(x_position['CE'] & angle_phi['VE'], steering_signal['ZE'])
rule19 = ctrl.Rule(x_position['RC'] & angle_phi['VE'], steering_signal['PM'])
rule20 = ctrl.Rule(x_position['RI'] & angle_phi['VE'], steering_signal['PM'])

rule21 = ctrl.Rule(x_position['LE'] & angle_phi['LV'], steering_signal['NB'])
rule22 = ctrl.Rule(x_position['LC'] & angle_phi['LV'], steering_signal['NM'])
rule23 = ctrl.Rule(x_position['CE'] & angle_phi['LV'], steering_signal['NS'])
rule24 = ctrl.Rule(x_position['RC'] & angle_phi['LV'], steering_signal['PS'])
rule25 = ctrl.Rule(x_position['RI'] & angle_phi['LV'], steering_signal['PM'])

rule26 = ctrl.Rule(x_position['LE'] & angle_phi['LU'], steering_signal['NB'])
rule27 = ctrl.Rule(x_position['LC'] & angle_phi['LU'], steering_signal['NB'])
rule28 = ctrl.Rule(x_position['CE'] & angle_phi['LU'], steering_signal['NM'])
rule29 = ctrl.Rule(x_position['RC'] & angle_phi['LU'], steering_signal['NS'])
rule30 = ctrl.Rule(x_position['RI'] & angle_phi['LU'], steering_signal['PS'])

rule31 = ctrl.Rule(x_position['LE'] & angle_phi['LB'], steering_signal['NB'])
rule32 = ctrl.Rule(x_position['LC'] & angle_phi['LB'], steering_signal['NB'])
rule33 = ctrl.Rule(x_position['CE'] & angle_phi['LB'], steering_signal['NM'])
rule34 = ctrl.Rule(x_position['RC'] & angle_phi['LB'], steering_signal['NM'])
rule35 = ctrl.Rule(x_position['RI'] & angle_phi['LB'], steering_signal['NS'])
# 創建模糊控制系統
steering_ctrl = ctrl.ControlSystem([rule1,rule2,rule3,rule4,rule5,
                                    rule6,rule7,rule8,rule9,rule10,
                                    rule11,rule12,rule13,rule14,rule15,
                                    rule16,rule17,rule18,rule19,rule20,
                                    rule21,rule22,rule23,rule24,rule25,
                                    rule26,rule27,rule28,rule29,rule30,
                                    rule31,rule32,rule33,rule34,rule35])

# 模糊控制器
steering = ctrl.ControlSystemSimulation(steering_ctrl)

###################################################
D2R=np.pi/180
#車輛座標to三角頂點
def car_coord_conver(car_x, car_y, car_φ):
    car_angle=car_φ*D2R
    #車輛三角形的原始座標
    triangle_coords = np.array([[car_x, car_y], [car_x - 2.5, car_y - 5], [car_x + 2.5, car_y - 5]])
    
    #旋轉矩陣
    rotation_center = triangle_coords[0]
    rotation_matrix = np.array([[np.sin(car_angle), -np.cos(car_angle)],
                                [np.cos(car_angle), np.sin(car_angle)]])
    
    #車輛三角形旋轉後的座標
    car_coords = np.dot(triangle_coords - rotation_center, rotation_matrix) + rotation_center
    
    return car_coords

#動畫更新函数
def update(num):#frame>num
    car_coords=car_coord_conver(X[num], Y[num], φ[num])
    #軌跡
    xdata.append(car_coords[0][0])
    ydata.append(car_coords[0][1])
    car_line.set_data(xdata, ydata)
    #車輛
    car_triangle.set_xy(car_coords)
    return car_triangle,car_line
###################################################
fig, ax = plt.subplots()
#[0, 100] x [0, 100]的水藍色方形
blue_square = Rectangle((0, 0), 100, 100, fc='lightblue')
ax.add_patch(blue_square)

# 終點(50, 100)
ax.plot(50,100, color='green', marker='o', markersize=2)#點

#計算路徑
X,Y,φ= np.zeros(frame_num+1),np.zeros(frame_num+1),np.zeros(frame_num+1)
X[0]=car_x
Y[0]=car_y
φ[0]=car_φ

for i in range(frame_num):
    #設定輸入值
    steering.input['X-position'] = X[i]
    steering.input['Angle'] = φ[i]
    #進行推論(質心解模糊法)
    steering.compute()
    #獲得輸出
    delta_angle = steering.output['Steering-angle']
    #計算位移
    φ[i+1]=φ[i]+delta_angle
    X[i+1]=X[i]+r*np.cos(φ[i+1]*D2R)
    Y[i+1]=Y[i]+r*np.sin(φ[i+1]*D2R)
    #顯示
    print('frame:',i,' X:',X[i],' Y:',Y[i],' φ:',φ[i],' θ:',delta_angle)
    #判斷是否到終點
    if(abs(Y[i+1])>100):
        frame_num=i+1
        #Docking error
        print('Docking error:',np.sqrt( ((φ[i]-90)/180)**2 + ((X[i]-50)/50)**2 + ((Y[i]-100)/100)**2 ))
        #Trajectory error
        print('Trajectory error:',r*i/np.sqrt( (X[0]-50)**2 + (Y[0]-100)**2 ))
        break


#車輛座標to三角
car_coords=car_coord_conver(car_x, car_y, car_φ)
#創建車輛的紅色三角形
car_triangle = Polygon(car_coords, closed=True, facecolor='red')
ax.add_patch(car_triangle)
xdata, ydata = [],[]
#軌跡
car_line, = ax.plot([], [],color='blue',lw=1)

#動畫
ani = FuncAnimation(fig, update, frames=frame_num, blit=True ,repeat=False,interval=1000/fps)

#顯示
plt.axis('scaled')
plt.show()
