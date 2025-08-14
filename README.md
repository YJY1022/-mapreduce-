# UAV分布式频谱感知系统仿真

本项目实现了完整的UAV分布式频谱感知系统MATLAB仿真，包含多UAV部署、A2G信道建模、PU Markov状态转移、协作检测等核心功能。

## 系统参数

基于技术要求，系统采用以下参数：

### 基本参数
- **采样频率**: 30.72 MHz (LTE/5G标准)
- **感知周期**: 6.5 μs
- **每周期采样数**: 200
- **UAV数量**: 10个分布式部署
- **仿真时隙**: 100个时隙

### UAV部署
- **水平距离**: (0, 1000] 米
- **飞行高度**: [300, 320] 米
- **随机角度部署**: 360°范围内随机分布

### PU参数
- **发射功率**: 20 dBm
- **状态模型**: 二状态Markov过程
- **平均活动时间**: 5时隙
- **平均空闲时间**: 10时隙

### A2G信道模型
- **LoS路径损耗指数**: 2 (自由空间)
- **NLoS路径损耗指数**: 4 (城市遮挡)
- **LoS路径损耗因子**: 0 dB
- **NLoS路径损耗因子**: -20 dB
- **环境常数**: A=9.6, B=0.16 (Al-Hourani模型)
- **小尺度衰落**: LoS用Rician (K=10dB), NLoS用Rayleigh

### 噪声参数
- **噪声功率谱密度**: -174 dBm/Hz
- **接收系统带宽**: 20 MHz
- **噪声因子**: 1 dB

## 文件说明

### 1. `uav_distributed_spectrum_sensing.m` ⭐
**主要仿真脚本** - 完整的UAV分布式频谱感知系统

**功能**:
- 10个UAV的分布式部署和位置建模
- A2G信道模型 (LoS/NLoS路径损耗和小尺度衰落)
- PU二状态Markov过程
- 多UAV接收信号生成和SNR计算
- 个体检测结果和性能分析
- 9个子图的综合可视化

**运行方法**:
```matlab
run('uav_distributed_spectrum_sensing.m')
```

**输出**:
- UAV 3D部署图
- PU Markov状态序列
- 各UAV SNR和检测性能
- LoS概率分析
- 信道增益时变特性
- 保存结果文件 `uav_distributed_sensing_results.mat`

### 2. `uav_cooperative_sensing.m` ⭐
**协作感知分析脚本** - 多UAV协作检测与决策融合

**功能**:
- 硬决策融合 (OR, AND, 多数投票)
- 软决策融合 (EGC, MRC, 距离加权)
- ROC曲线分析
- 协作增益评估
- 时变性能分析
- 12个子图的全面性能比较

**运行方法**:
```matlab
% 首先运行分布式感知
run('uav_distributed_spectrum_sensing.m')
% 然后运行协作分析
run('uav_cooperative_sensing.m')
```

**输出**:
- 个体vs协作检测性能比较
- ROC曲线对比
- 融合权重分析
- 协作增益量化
- 保存结果文件 `uav_cooperative_sensing_results.mat`

### 3. `uav_communication_simulation.m`
**基础通信仿真** - 单链路通信系统仿真 (兼容原始需求)

**功能**:
- QPSK调制/解调
- 过采样信号处理
- 时域/频域/功率谱密度分析

### 4. `uav_communication_simulation_fixed.m`
**修复版通信仿真** - 解决数值计算问题的版本

### 5. `demo_uav_simulation.m`
**演示脚本** - 简化版本，便于快速验证

### 6. `uav_parameter_analysis.m`
**参数分析脚本** - 过采样因子性能比较

## 图表说明

### 时域分析
- **射频信号时域图**: 显示载波调制后的接收信号
- **基带信号时域图**: 显示I/Q分量的时域波形
- **星座图**: 显示QPSK符号的分布

### 频域分析
- **射频信号频谱**: 显示围绕载波频率的频谱分布
- **基带信号频谱**: 显示基带信号的频谱特性
- **功率谱密度**: 使用Welch方法估计的PSD

### 性能分析
- **SNR**: 信噪比计算
- **EVM**: 误差向量幅度
- **PAPR**: 峰均功率比
- **频谱效率**: bps/Hz
- **频谱泄漏**: 带外泄漏百分比

## 参数调整

可以通过修改以下参数来适应不同的仿真需求：

```matlab
% 系统参数
fc = 30.72e6;           % 载波频率
T_sense = 63e-6;        % 感知周期
oversample_factor = 8;  % 过采样因子

% 信号参数
data_rate = 1e6;        % 数据速率
P_tx = 20;              % 发射功率 (dBm)

% 信道参数
path_loss_los = 96;     % LoS路径损耗 (dB)
path_loss_nlos = 110;   % NLoS路径损耗 (dB)
K_factor = 10;          % Rician K因子 (dB)
```

## 运行要求

- MATLAB R2018b 或更高版本
- Signal Processing Toolbox (用于滤波器设计)
- Communications Toolbox (用于调制解调函数)

## 快速开始

### 推荐运行顺序：

1. **分布式感知仿真**: 
   ```matlab
   run('uav_distributed_spectrum_sensing.m')
   ```
   - 生成10个UAV的分布式频谱感知系统仿真
   - 包含完整的A2G信道建模和PU Markov过程

2. **协作感知分析**:
   ```matlab
   run('uav_cooperative_sensing.m')
   ```
   - 基于步骤1的结果进行协作检测分析
   - 比较多种融合算法的性能

3. **基础通信仿真** (可选):
   ```matlab
   run('uav_communication_simulation_fixed.m')
   ```
   - 单链路通信系统仿真
   - 过采样条件下的信号分析

### 核心功能验证：
- **多UAV部署**: 3D空间随机部署，符合高度和距离约束
- **A2G信道**: LoS/NLoS概率建模，路径损耗和小尺度衰落
- **Markov过程**: PU状态转移，活动/空闲时间统计
- **协作检测**: 硬决策和软决策融合算法
- **性能评估**: SNR、检测率、虚警率、ROC曲线

## 结果解读

### 过采样的影响
- 更高的过采样因子提供更好的信号重建精度
- 但增加了计算复杂度和存储需求
- 建议根据精度要求选择合适的过采样因子

### 信道条件对比
- LoS信道通常提供更好的SNR和更低的EVM
- NLoS信道由于多径衰落导致性能下降
- Rician K因子影响LoS信道的性能

## 技术特点

1. **符合标准**: 基于LTE-5G技术标准
2. **实际参数**: 使用真实的UAV通信系统参数
3. **全面分析**: 涵盖时域、频域和性能指标
4. **灵活配置**: 支持参数调整和扩展
5. **可视化**: 提供直观的图表分析

## 扩展建议

- 添加更复杂的信道模型 (如移动信道)
- 实现自适应调制编码
- 加入多天线MIMO技术
- 添加信道编码和译码
- 实现实时信号处理