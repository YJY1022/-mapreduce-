# UAV-PU通信系统仿真

这是一个完整的MATLAB仿真程序，用于仿真无人机(UAV)和主用户(PU)之间的通信系统。仿真包含A2G信道模型、马尔可夫PU行为建模以及完整的信号处理流程。

## 仿真特性

### 1. 系统模型
- **UAV分布**: 10个分布式部署的UAV，位置随机分布
  - 水平距离：0-1000米
  - 高度：300-320米
- **PU模型**: 单个主用户，位于原点
- **载波频率**: 2.4 GHz
- **系统带宽**: 20 MHz

### 2. A2G信道模型
- **路径损耗模型**: 
  - LoS路径损耗指数：2（自由空间）
  - NLoS路径损耗指数：4（城市遮挡）
- **LoS概率**: Al-Hourani模型，P_LoS = 1/(1+a*exp(-b*(θ-a)))
  - 环境常数：a=9.6, b=0.16
- **小尺度衰落**:
  - LoS：Nakagami-m衰落（m=2）
  - NLoS：Rayleigh衰落

### 3. PU行为建模
- **马尔可夫模型**: 二状态马尔可夫链（空闲/活动）
- **平均活动时间**: 5时隙
- **平均空闲时间**: 10时隙
- **自动计算**: 状态转移概率和稳态概率

### 4. 信号处理流程
1. **数据生成**: QPSK调制符号
2. **成型滤波**: 升余弦滤波器（滚降因子0.35）
3. **上变频**: 基带信号调制到载波频率
4. **信道传输**: 应用信道增益和传播延迟
5. **下变频**: 射频信号恢复到基带
6. **匹配滤波**: 最大化信噪比
7. **添加噪声**: 高斯白噪声

### 5. 噪声模型
- **噪声功率谱密度**: -174 dBm/Hz
- **噪声因子**: 1 dB
- **系统带宽**: 20 MHz
- **自动SNR计算**: 包含理论性能分析

## 文件结构

```
/workspace/
├── uav_pu_simulation.m      # 主仿真脚本
├── a2g_channel_model.m      # A2G信道模型函数
├── markov_pu_model.m        # PU马尔可夫模型函数
├── signal_processing.m      # 信号处理流程函数
├── snr_calculation.m        # SNR计算函数
├── visualization.m          # 结果可视化函数
└── README.md               # 说明文档
```

## 使用方法

### 1. 运行完整仿真
```matlab
% 在MATLAB命令窗口中运行
run('/workspace/uav_pu_simulation.m')
```

### 2. 单独使用各个模块
```matlab
% 加载系统参数
load('/workspace/simulation_params.mat');

% 使用A2G信道模型
channel_params.a = 9.6;
channel_params.b = 0.16;
channel_params.los_path_loss_exp = 2;
channel_params.nlos_path_loss_exp = 4;
[channel_gains, los_prob, path_loss_db] = a2g_channel_model(distances, elevation_angles, fc, channel_params);

% 生成PU状态
[pu_states, transition_matrix] = markov_pu_model(5, 10, 1000);
```

## 仿真输出

### 1. 控制台输出
- 系统参数初始化信息
- UAV位置和距离统计
- 信道模型计算结果
- 马尔可夫链参数和统计
- 信号处理流程状态
- SNR计算和性能分析
- 仿真总结

### 2. 可视化图表

#### 图表1: UAV和PU位置分布（3D散点图）
- 显示UAV和PU的空间分布关系

#### 图表2: SNR分布分析
- SNR分布直方图
- 各UAV的SNR对比

#### 图表3: 时域波形图（6个子图）
- PU发射基带信号（实部/虚部）
- PU发射射频信号
- UAV接收基带信号（实部/虚部）
- UAV接收射频信号

#### 图表4: 频域频谱图（4个子图）
- 发射/接收射频信号频谱
- 发射/接收基带信号频谱

#### 图表5: 功率谱密度图（4个子图）
- 发射/接收射频信号PSD
- 发射/接收基带信号PSD

#### 图表6: PU状态时序图
- PU活动状态时序
- 接收信号与PU状态对应关系

#### 图表7: QPSK调制星座图
- 发射星座图
- 接收星座图

## 参数配置

### 系统参数
```matlab
num_uav = 10;                    % UAV数量
fc = 2.4e9;                      % 载波频率
pu_tx_power_dbm = 30;            % PU发射功率
bandwidth = 20e6;                % 系统带宽
fs = 40e6;                       % 采样频率
```

### 信道参数
```matlab
los_path_loss_exp = 2;           % LoS路径损耗指数
nlos_path_loss_exp = 4;          % NLoS路径损耗指数
a = 9.6; b = 0.16;              % Al-Hourani模型参数
```

### 马尔可夫参数
```matlab
active_time_slots = 5;           % 平均活动时间
idle_time_slots = 10;            % 平均空闲时间
```

## 典型仿真结果

### 性能指标
- **平均LoS概率**: ~0.4-0.6
- **平均路径损耗**: ~100-120 dB
- **平均SNR**: 根据距离和信道条件变化
- **PU活动占比**: ~33%（基于马尔可夫参数）

### SNR性能分析
- **优秀链路** (≥10 dB): 通常1-3个UAV
- **良好链路** (0-10 dB): 通常4-6个UAV  
- **较差链路** (<0 dB): 通常1-5个UAV

## 扩展功能

### 1. 参数调整
可以轻松修改各种仿真参数：
- UAV数量和位置分布
- 信道模型参数
- PU行为模式
- 信号处理参数

### 2. 性能分析
代码提供了完整的性能分析功能：
- 理论BER计算
- 香农容量分析
- SNR统计分析
- 链路质量评估

### 3. 可视化定制
可以根据需要调整可视化参数：
- 显示时间范围
- 频率范围
- 图表样式

## 系统要求

- MATLAB R2018b或更高版本
- Signal Processing Toolbox
- Communications Toolbox

## 注意事项

1. 仿真使用随机种子确保结果可重现
2. 大型仿真可能需要较长计算时间
3. 可视化图表数量较多，建议分批查看
4. 参数修改后需要重新运行完整仿真

## 技术支持

如有问题，请检查：
1. MATLAB版本兼容性
2. 所需工具箱是否安装
3. 文件路径是否正确
4. 参数设置是否合理