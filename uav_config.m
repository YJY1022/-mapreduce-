function config = uav_config()
%% UAV通信系统参数配置文件
% 根据图片中的技术规格设置系统参数
% 用户可以修改这些参数来改变仿真场景

%% 基本通信参数
config.communication = struct();
config.communication.fc = 30.72e9;           % 载波频率 (Hz) - 基于图片中30.72MHz的1000倍(毫米波)
config.communication.symbol_rate = 1e6;      % 符号速率 (sps)
config.communication.modulation = 'QPSK';    % 调制方式
config.communication.rolloff = 0.25;         % 根升余弦滤波器滚降因子
config.communication.filter_span = 8;        % 滤波器长度（符号周期）

%% UAV几何参数（基于图片场景）
config.geometry = struct();
config.geometry.uav_height = 100;            % UAV高度 (m)
config.geometry.uav_speed = 50;              % UAV速度 (m/s)
config.geometry.distance_2d = 500;           % 基站到UAV的水平距离 (m)
config.geometry.base_station_height = 10;    % 基站高度 (m)

%% 信道参数（基于图片中的LoS/NLoS模型）
config.channel = struct();
% LoS信道参数（图片中显示的A=9.6, B=0.16）
config.channel.los_A = 9.6;                  % LoS路径损耗参数A
config.channel.los_B = 0.16;                 % LoS环境因子B
config.channel.nlos_A = 9.6;                 % NLoS路径损耗参数（可调整）
config.channel.nlos_B = 0.16;                % NLoS环境因子（可调整）
config.channel.path_loss_exp = 2;            % 路径损耗指数
config.channel.SNR_dB = 20;                  % 信噪比 (dB)

%% 仿真参数
config.simulation = struct();
config.simulation.num_symbols = 1000;        % 仿真符号数
config.simulation.oversample_factor = 4;     % 过采样倍数
config.simulation.enable_doppler = true;     % 是否启用多普勒效应
config.simulation.enable_multipath = false;  % 是否启用多径效应
config.simulation.seed = 42;                 % 随机种子

%% 频率参数（基于图片中的频谱信息）
config.frequency = struct();
config.frequency.bandwidth = 20e6;           % 信号带宽 (Hz) - 基于图片中20MHz
config.frequency.guard_band = 1e6;           % 保护带宽 (Hz)

%% 天线参数
config.antenna = struct();
config.antenna.gain_tx = 10;                 % 发射天线增益 (dBi)
config.antenna.gain_rx = 10;                 % 接收天线增益 (dBi)
config.antenna.polarization = 'vertical';    % 极化方式

%% 功率参数
config.power = struct();
config.power.tx_power_dBm = 30;             % 发射功率 (dBm)
config.power.noise_figure_dB = 6;           % 噪声系数 (dB)

%% 显示配置信息
fprintf('=== UAV通信系统配置 ===\n');
fprintf('载波频率: %.2f GHz\n', config.communication.fc/1e9);
fprintf('符号速率: %.2f Msps\n', config.communication.symbol_rate/1e6);
fprintf('UAV高度: %d m\n', config.geometry.uav_height);
fprintf('水平距离: %d m\n', config.geometry.distance_2d);
fprintf('信噪比: %d dB\n', config.channel.SNR_dB);
fprintf('信号带宽: %.2f MHz\n', config.frequency.bandwidth/1e6);

end