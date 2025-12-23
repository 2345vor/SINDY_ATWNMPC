# 自适应轨迹跟踪NMPC控制系统

## 项目概述

本项目实现了一个基于识别模型的自适应轨迹跟踪非线性模型预测控制（NMPC）系统，主要用于无人船或水下机器人等移动平台的精确路径跟踪控制。系统基于CasADi框架构建优化问题，使用识别的动力学模型预测系统未来状态，并通过自适应权重调整机制提高跟踪精度和鲁棒性。

youtube video:

<iframe width="560" height="315" src="https://www.youtube.com/embed/amLDn78ToMA?si=16PgnXkkBhMib3H5" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

<iframe width="560" height="315" src="https://www.youtube.com/embed/GuF1A3Jz97g?si=NNRSXAfqiHLsGjHT" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

Bilibili video:

<iframe src="https://player.bilibili.com/player.html?isOutside=true&aid=115633102525246&bvid=BV1iVSFB7ESe&cid=34365508342&p=1" scrolling="no" border="0" frameborder="no" framespacing="0" allowfullscreen="true"></iframe>

<iframe src="https://player.bilibili.com/player.html?isOutside=true&aid=115633102525239&bvid=BV1iVSFB7ESr&cid=34365442883&p=1" scrolling="no" border="0" frameborder="no" framespacing="0" allowfullscreen="true"></iframe>

## 核心功能特点

### 1. 多模型支持

- 提供三种不同的识别动力学模型（Model 1-3）
- 支持基于SGF、EKF和LPF方法识别的模型参数
- 灵活的模型选择机制，可通过命令行参数快速切换

### 2. 多样化轨迹跟踪

- 支持椭圆轨迹（x = 150*sin(t) + 28.2, y = 60*cos(t) - 85.4）
- 支持正弦直线轨迹（x = 40*sin(t) + 1, y = 10*t）
- 支持双正弦轨迹（x = 150*sin(t) + 28.2, y = 60*cos(2*t) - 85.4）

### 3. 自适应控制机制

- 根据距离误差动态调整权重参数
- 支持权重参数的上限和下限约束
- 可通过命令行参数启用/禁用自适应功能

### 4. 约束管理

- 对状态变量（u, v, r, x, y）施加边界约束
- 对控制输入（推进器力矩）施加限制
- 灵活的约束参数配置

### 5. 噪声处理与鲁棒性

- 支持添加高斯白噪声模拟真实环境
- 可配置噪声均值和标准差
- 通过优化权重参数提高抗干扰能力

### 6. 可视化与数据记录

- 自动生成轨迹跟踪对比图
- 提供误差分析和性能统计
- 记录完整的状态变量、控制输入和误差数据

## 技术实现

### 1. 系统架构

```
├── 命令行参数解析
├── 轨迹生成
├── 动力学模型构建
├── NMPC控制器实现
├── 自适应权重调整
├── 仿真循环
├── 结果可视化
└── 数据保存
```

### 2. 数学模型

系统状态向量：

- `u`: 前进速度
- `v`: 横移速度
- `r`: 横摇角速度
- `x_pos`: x方向位置
- `y_pos`: y方向位置
- `psi`: 航向角

控制输入：

- `Tp`: 左推进器力矩
- `Ts`: 右推进器力矩

### 3. NMPC控制算法

1. **预测模型**：使用识别的动力学模型预测未来N步的系统状态
2. **目标函数**：综合考虑状态误差和控制输入，通过权重矩阵进行调节
3. **约束条件**：包括状态变量约束和控制输入约束
4. **优化求解**：使用IPOPT求解器求解二次规划问题
5. **滚动优化**：仅使用第一个控制输入，然后重新求解优化问题

### 4. 自适应权重调整

根据当前位置误差动态调整权重参数：

```python
distance_error = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
distance_coeff = max(0, min(1.0, distance_error / eta))
F_current = [F_low[i] + distance_coeff * (F_high[i] - F_low[i]) for i in range(6)]
```

## 安装依赖

### 必要的Python包

```bash
pip install numpy
pip install casadi
pip install matplotlib
pip install pandas
```

### 环境要求

- Python 3.6+
- CasADi 3.5.5+
- NumPy 1.18+
- Matplotlib 3.3+
- Pandas 1.0+

## 使用方法

### 基本用法

```bash
python identified_model_nmpc_test.py
```

### 命令行参数详解

| 参数               | 类型      | 默认值       | 描述                                                               |
| ------------------ | --------- | ------------ | ------------------------------------------------------------------ |
| `--model`        | int (1-3) | 1            | 模型类型 (1: 基础模型18参数, 2: 分离模型21参数, 3: 简化模型16参数) |
| `--trajectory`   | int (1-3) | 1            | 跟踪轨迹 (1: 椭圆, 2: 正弦直线, 3: 双正弦)                         |
| `--predict_step` | int       | 10           | 预测步长,最大值20,最小值5,默认10                                   |
| `--dt`           | float     | 0.1          | 采样时间,最大值0.5,最小值0.05,默认0.1                              |
| `--cycle_time`   | int       | 45           | 轨迹周期时间,最大值90,最小值45,默认45,推荐是3的倍数                |
| `--loop_num`     | int       | 1            | 循环次数,最大值5,最小值1,默认1                                     |
| `--noise_mean`   | float     | -1           | 轨迹噪声均值                                                       |
| `--noise_std`    | float     | 0.52         | 轨迹噪声标准差                                                     |
| `--eta`          | float     | 100.0        | 自适应NMPC控制参数                                                 |
| `--adaptive`     | flag      | True         | 是否启用自适应NMPC控制                                             |
| `--output_dir`   | string    | nmpc_results | 输出目录                                                           |

### 仿真使用示例

1. 使用模型2跟踪双正弦轨迹，并启用自适应控制：

```bash
python identified_model_nmpc_test.py --model 2 --trajectory 3 --adaptive
```

2. 使用模型1跟踪椭圆轨迹，调整预测步长为15：

```bash
python identified_model_nmpc_test.py --model 1 --trajectory 1 --predict_step 15
```

3. 使用模型3跟踪正弦直线轨迹，修改噪声参数：

```bash
python identified_model_nmpc_test.py --model 3 --trajectory 2 --noise_mean 0 --noise_std 0.3
```

4. 禁用自适应控制，使用默认参数：

```bash
python identified_model_nmpc_test.py --no-adaptive
```

### 下位机使用示例

本项目还提供下位机ESP32驱动代码，用于与下位机进行通信和数据处理。
下位机驱动代码包含在 `esp32_mini_nmpc_boat`目录下，整体框架基于Platformio的Arduino开发环境。核心代码在 `esp32_mini_nmpc_boat/src/main.cpp`中。Platformio依赖库在 `esp32_mini_nmpc_boat\platformio.ini`中。主要功能包括：
下位机esp32外接TB6612FNG两路电机驱动模块，用于控制船体的运动。

- 与上位机通过UDP通信
- 接收和解析上位机发送的控制指令
- 处理控制输入
  服务器地址：192.168.0.102
  UDP端口：8000
  解析控制指令 格式：speed1,speed2,speed3,speed4；
  前进实例：500,0,500,0；
  后退实例：-500,0,-500,0；
  左转实例：0,500,0,-500；
  右转实例：0,-500,0,500；

## 输出结果说明

### 1. 控制台输出

- 轨迹点数量和轨迹参数
- 自适应控制状态和模型类型
- 初始状态和目标轨迹起点
- 仿真进度和完成提示
- 跟踪性能统计（平均误差、最大误差等）

### 2. 可视化结果

系统自动生成以下可视化图表并保存到输出目录：

1. **轨迹跟踪对比图**：显示参考轨迹和NMPC跟踪轨迹的对比
2. **跟踪误差分析图**：包括横向误差和航向误差的时间历程
3. **状态变量图**：展示u, v, r和psi的变化情况
4. **推进器输出图**：显示左右推进器的力矩输出

### 3. 数据文件

生成的CSV文件包含以下数据：

- 时间序列
- 实际位置（X, Y）
- 参考位置（Ref_X, Ref_Y）
- 状态变量（u, v, r, psi）
- 控制输入（Tp, Ts）
- 误差数据（Lateral_Error, Heading_Error）

## 引用

如果您在研究中使用了此代码，请引用我们的工作：

```
@article{Pengliu2025ATWNMPC,
  title={Sparse Identification of Nonlinear Dynamics with Adaptive Terminal Weight NMPC in Unmanned Surface Vehicle},
  author={Peng Liu, Yunsheng Fan,~\IEEEmembership{Member, IEEE}, Yan Wang, Xiaojie Sun, Zhe Sun, and Quan An},
  journal={Submitted to IEEE Transactions on Control Systems Technology},
  year={2025}
}
```

## Contact

For questions or collaborations:

刘鹏: 3126171871@qq.com

Repository: https://github.com/2345vor/SINDY_ATWNMPC
