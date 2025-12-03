# Adaptive Trajectory Tracking NMPC Control System

## Project Overview

This project implements an adaptive trajectory tracking Nonlinear Model Predictive Control (NMPC) system based on identified models, primarily used for precise path tracking control of mobile platforms such as unmanned surface vehicles or underwater robots. The system builds optimization problems based on the CasADi framework, uses identified dynamic models to predict future system states, and improves tracking accuracy and robustness through an adaptive weight adjustment mechanism.

## Core Features

### 1. Multi-model Support

- Provides three different identified dynamic models (Model 1-3)
- Supports model parameters identified based on SGF, EKF, and LPF methods
- Flexible model selection mechanism, quickly switchable via command-line parameters

### 2. Diverse Trajectory Tracking

- Supports elliptical trajectory (x = 150*sin(t) + 28.2, y = 60*cos(t) - 85.4)
- Supports sinusoidal straight-line trajectory (x = 40*sin(t) + 1, y = 10*t)
- Supports double sine trajectory (x = 150*sin(t) + 28.2, y = 60*cos(2*t) - 85.4)

### 3. Adaptive Control Mechanism

- Dynamically adjusts weight parameters based on distance error
- Supports upper and lower bound constraints for weight parameters
- Can enable/disable adaptive functionality via command-line parameters

### 4. Constraint Management

- Applies boundary constraints to state variables (u, v, r, x, y)
- Implements limits on control inputs (thruster torques)
- Flexible constraint parameter configuration

### 5. Noise Handling and Robustness

- Supports adding Gaussian white noise to simulate real environments
- Configurable noise mean and standard deviation
- Improves anti-interference capability by optimizing weight parameters

### 6. Visualization and Data Recording

- Automatically generates trajectory tracking comparison charts
- Provides error analysis and performance statistics
- Records complete state variables, control inputs, and error data

## Technical Implementation

### 1. System Architecture

```
├── Command Line Parameter Parsing
├── Trajectory Generation
├── Dynamic Model Construction
├── NMPC Controller Implementation
├── Adaptive Weight Adjustment
├── Simulation Loop
├── Result Visualization
└── Data Saving
```

### 2. Mathematical Model

System state vector:

- `u`: Surge velocity
- `v`: Sway velocity
- `r`: Yaw angular velocity
- `x_pos`: Position in x direction
- `y_pos`: Position in y direction
- `psi`: Heading angle

Control inputs:

- `Tp`: Left thruster torque
- `Ts`: Right thruster torque

### 3. NMPC Control Algorithm

1. **Prediction Model**: Uses identified dynamic models to predict future N steps of system states
2. **Objective Function**: Considers state errors and control inputs comprehensively, adjusted through weight matrices
3. **Constraint Conditions**: Includes state variable constraints and control input constraints
4. **Optimization Solution**: Uses IPOPT solver to solve quadratic programming problems
5. **Receding Horizon Optimization**: Only uses the first control input, then resolves the optimization problem

### 4. Adaptive Weight Adjustment

Dynamically adjusts weight parameters based on current position error:

```python
distance_error = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
distance_coeff = max(0, min(1.0, distance_error / eta))
F_current = [F_low[i] + distance_coeff * (F_high[i] - F_low[i]) for i in range(6)]
```

## Installation Dependencies

### Required Python Packages

```bash
pip install numpy
pip install casadi
pip install matplotlib
pip install pandas
```

### Environment Requirements

- Python 3.6+
- CasADi 3.5.5+
- NumPy 1.18+
- Matplotlib 3.3+
- Pandas 1.0+

## Usage

### Basic Usage

```bash
python identified_model_nmpc_test.py
```

### Command Line Parameter Details

| Parameter           | Type      | Default Value | Description                                                         |
| ------------------- | --------- | ------------- | ------------------------------------------------------------------- |
| `--model`           | int (1-3) | 1             | Model type (1: Basic model with 18 parameters, 2: Separated model with 21 parameters, 3: Simplified model with 16 parameters) |
| `--trajectory`      | int (1-3) | 1             | Tracking trajectory (1: Ellipse, 2: Sinusoidal straight line, 3: Double sine) |
| `--predict_step`    | int       | 10            | Prediction horizon, maximum 20, minimum 5, default 10               |
| `--dt`              | float     | 0.1           | Sampling time, maximum 0.5, minimum 0.05, default 0.1               |
| `--cycle_time`      | int       | 45            | Trajectory cycle time, maximum 90, minimum 45, default 45, recommended to be a multiple of 3 |
| `--loop_num`        | int       | 1             | Number of loops, maximum 5, minimum 1, default 1                    |
| `--noise_mean`      | float     | -1            | Trajectory noise mean                                               |
| `--noise_std`       | float     | 0.52          | Trajectory noise standard deviation                                 |
| `--eta`             | float     | 100.0         | Adaptive NMPC control parameter                                     |
| `--adaptive`        | flag      | True          | Whether to enable adaptive NMPC control                             |
| `--output_dir`      | string    | nmpc_results  | Output directory                                                    |

### Usage Examples

1. Use Model 2 to track double sine trajectory with adaptive control enabled:

```bash
python identified_model_nmpc_test.py --model 2 --trajectory 3 --adaptive
```

2. Use Model 1 to track elliptical trajectory with prediction horizon adjusted to 15:

```bash
python identified_model_nmpc_test.py --model 1 --trajectory 1 --predict_step 15
```

3. Use Model 3 to track sinusoidal straight line trajectory with modified noise parameters:

```bash
python identified_model_nmpc_test.py --model 3 --trajectory 2 --noise_mean 0 --noise_std 0.3
```

4. Disable adaptive control and use default parameters:

```bash
python identified_model_nmpc_test.py --no-adaptive
```

## Output Results Description

### 1. Console Output

- Number of trajectory points and trajectory parameters
- Adaptive control status and model type
- Initial state and target trajectory starting point
- Simulation progress and completion prompts
- Tracking performance statistics (average error, maximum error, etc.)

### 2. Visualization Results

The system automatically generates the following visualization charts and saves them to the output directory:

1. **Trajectory Tracking Comparison Chart**: Shows comparison between reference trajectory and NMPC tracking trajectory
2. **Tracking Error Analysis Chart**: Includes time history of lateral error and heading error
3. **State Variables Chart**: Displays changes in u, v, r, and psi
4. **Thruster Output Chart**: Shows torque outputs of left and right thrusters

### 3. Data Files

The generated CSV files include the following data:

- Time series
- Actual position (X, Y)
- Reference position (Ref_X, Ref_Y)
- State variables (u, v, r, psi)
- Control inputs (Tp, Ts)
- Error data (Lateral_Error, Heading_Error)

## Citation
If you use this code in your research, please cite our work:
```
@article{delima2025colregs,
  title={Sparse Identification of Nonlinear Dynamics with Adaptive Terminal Weight NMPC in Unmanned Surface Vehicle},
  author={Peng Liu, Yunsheng Fan,~\IEEEmembership{Member, IEEE}, Yan Wang, Xiaojie Sun, Zhe Sun, and Quan An},
  journal={Submitted to IEEE Sensors Journal},
  year={2025}
}
```
## Acknowledgments
This work was partially supported by:

College of Marine Electrical Engineering, Dalian Maritime University, Dalian 116026, China.
Key Laboratory of Technology and System for Intelligent Ships of Liaoning Province, Dalian 116026, China.
College of Information Engineering, Henan University of Science and Technology, Henan 471023, China.
School of Automation and Intelligent Sensing, Shanghai Jiao Tong University, Shanghai 200240, China.
## License
This project is open-source and available under the terms specified in the repository.

## Contact
For questions or collaborations:

Peng Liu: 3126171871@qq.com

Repository: https://github.com/2345vor/SINDY_ATWNMPC

