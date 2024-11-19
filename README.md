# 倒立摆控制仿真（Python环境）

## 用法
1. 配置环境
   > 终端执行：conda env create -f inverted_pendulum_sim.yaml
2. 运行仿真
   > 解除simulator.py中所选择的控制器代码的注释，运行simulator.py

## 运行效果
- 不加控制
![不加控制](assets/NoControl.gif)
![不加控制](assets/NoControl.png)
- 极点配置
![极点配置](assets/PolePlacementController.gif)
![极点配置](assets/PolePlacementController.png)
- 带观测器的极点配置
![控制曲线](assets/PolePlacementControllerWithObserverControlRes.png)
![观测器误差曲线](assets/PolePlacementControllerWithObserverEstimateRes.png)
- LQR
![LQR](assets/LQRController.gif)
![LQR](assets/LQRController.png)
- 角度PID
![角度PID](assets/PidController1.gif)
![角度PID](assets/PidController1.png)
- 串级PID,角度环+角速度环
![串级PID,角度环+角速度环](assets/PidController2.gif)
![串级PID,角度环+角速度环](assets/PidController2.png)
- 串级PID,位置环+角度环+角速度环
![串级PID,位置环+角度环+角速度环](assets/PidController3.gif)
![串级PID,位置环+角度环+角速度环](assets/PidController3.png)
- MPC
![MPC](assets/MPCController.gif)
![MPC](assets/MPCController.png)