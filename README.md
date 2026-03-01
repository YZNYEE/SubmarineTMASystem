# TMA (Target Motion Analysis) Project

这是一个用于目标运动分析 (Target Motion Analysis, TMA) 的 C++ 项目。项目包含了多种估计算法，旨在通过观测数据（如方位角、频率等）对目标的运动状态（位置、速度）进行估计。

## 项目结构

**注意：目前仅集成了 MLE (最大似然估计) 算法，其他模块 (EKELUND, PF, UKF) 正在开发中，将陆续集成。**

- **COMMON**: 公共数据结构和常量定义。
- **EKELUND**: (待集成) Ekelund 测距算法实现。
- **MLE**: (已集成) 最大似然估计 (Maximum Likelihood Estimation) 算法，基于 Ceres Solver 进行非线性最小二乘优化。
- **PF**: (待集成) 粒子滤波 (Particle Filter) 算法实现。
- **UKF**: (待集成) 无迹卡尔曼滤波 (Unscented Kalman Filter) 算法实现。

## 依赖库

本项目依赖以下第三方库，请确保在构建前已正确安装：

1.  **Ceres Solver**
    - 用于解决非线性最小二乘问题，主要在 MLE 模块中使用。
    - [官方网站](http://ceres-solver.org/)

2.  **Eigen3**
    - C++ 模板库，用于线性代数运算。Ceres Solver 强依赖于 Eigen。
    - [官方网站](https://eigen.tuxfamily.org/)

## 构建说明

本项目使用 CMake 进行构建。

### 1. 配置第三方库路径

如果你的第三方库未安装在系统标准路径下，可以通过设置 `TMA_3RD_PARTY_DIR` 变量来指定查找路径，或者设置标准的 `Ceres_DIR` 和 `Eigen3_DIR`。

默认情况下，项目会在 `${CMAKE_SOURCE_DIR}/3rdparty` 下查找库。

### 2. 编译步骤

```bash
mkdir build
cd build
cmake ..
cmake --build .
```

## 使用示例

编译成功后，将生成 `TMA` 可执行文件。你可以运行该程序来验证各模块的功能。