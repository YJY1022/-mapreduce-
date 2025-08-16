%% UAV-PU 通信系统仿真
% 仿真UAV和主用户(PU)的通信系统，包含A2G信道模型、马尔可夫PU行为建模
% 以及完整的信号处理流程
clear; close all; clc;

%% 1. 系统参数初始化
fprintf('正在初始化系统参数...\n');

% 基本参数
num_uav = 10;                    % UAV数量
fc = 2.4e9;                      % 载波频率 (2.4 GHz)
c = 3e8;                         % 光速 (m/s)
lambda = c / fc;                 % 波长

% UAV位置参数
min_horizontal_dist = 0;         % 最小水平距离 (m)
max_horizontal_dist = 1000;      % 最大水平距离 (m)
min_height = 300;                % 最小高度 (m)
max_height = 320;                % 最大高度 (m)

% PU参数
pu_tx_power_dbm = 30;            % PU发射功率 (dBm)
pu_tx_power = 10^((pu_tx_power_dbm - 30)/10); % 转换为瓦特

% 信道模型参数
los_path_loss_exp = 2;           % LoS路径损耗指数
nlos_path_loss_exp = 4;          % NLoS路径损耗指数
a = 9.6;                         % 环境常数a (Al-Hourani模型)
b = 0.16;                        % 环境常数b (Al-Hourani模型)

% 噪声参数
noise_psd_dbm_hz = -174;         % 噪声功率谱密度 (dBm/Hz)
bandwidth = 20e6;                % 系统带宽 (20 MHz)
noise_figure_db = 1;             % 噪声因子 (dB)

% 马尔可夫模型参数
active_time_slots = 5;           % 平均活动时间槽
idle_time_slots = 10;            % 平均空闲时间槽

% 信号处理参数
fs = 40e6;                       % 采样频率 (40 MHz)
symbol_rate = 1e6;               % 符号速率 (1 Msps)
oversample_factor = 4;           % 过采样因子
num_symbols = 1000;              % 符号数量
rolloff_factor = 0.35;           % 升余弦滤波器滚降因子

%% 2. UAV位置分布
fprintf('生成UAV位置...\n');
rng(42); % 设置随机种子以保证结果可重现

% 随机生成UAV位置
uav_x = min_horizontal_dist + (max_horizontal_dist - min_horizontal_dist) * rand(num_uav, 1);
uav_y = min_horizontal_dist + (max_horizontal_dist - min_horizontal_dist) * rand(num_uav, 1);
uav_z = min_height + (max_height - min_height) * rand(num_uav, 1);

% PU位置 (固定在原点地面)
pu_x = 0; pu_y = 0; pu_z = 0;

% 计算UAV到PU的距离
distances = sqrt((uav_x - pu_x).^2 + (uav_y - pu_y).^2 + (uav_z - pu_z).^2);

% 计算仰角 (elevation angle)
elevation_angles = asind(uav_z ./ distances);

fprintf('UAV部署完成，距离范围: %.1f - %.1f 米\n', min(distances), max(distances));

%% 3. 显示UAV分布
figure('Position', [100, 100, 800, 600]);
scatter3(uav_x, uav_y, uav_z, 100, 'filled', 'MarkerFaceColor', 'blue');
hold on;
scatter3(pu_x, pu_y, pu_z, 150, 'filled', 'MarkerFaceColor', 'red', 'Marker', '^');
xlabel('X (m)'); ylabel('Y (m)'); zlabel('高度 (m)');
title('UAV和PU位置分布');
legend('UAV', 'PU', 'Location', 'best');
grid on; axis equal;

% 保存参数到工作空间，供后续函数使用
save('/workspace/simulation_params.mat');
fprintf('系统参数初始化完成！\n\n');

%% 4. A2G信道模型计算
fprintf('=== 步骤2: 计算A2G信道模型 ===\n');

% 准备信道参数结构体
channel_params.a = a;
channel_params.b = b;
channel_params.los_path_loss_exp = los_path_loss_exp;
channel_params.nlos_path_loss_exp = nlos_path_loss_exp;

% 计算信道增益
[channel_gains, los_prob, path_loss_db] = a2g_channel_model(distances, elevation_angles, fc, channel_params);

%% 5. PU马尔可夫模型
fprintf('=== 步骤3: 生成PU马尔可夫模型 ===\n');

% 仿真时间槽数 (基于符号数量)
total_time_slots = num_symbols;

% 生成PU状态序列
[pu_states, transition_matrix] = markov_pu_model(active_time_slots, idle_time_slots, total_time_slots);

%% 6. 计算传播延迟
fprintf('=== 步骤4: 计算传播延迟 ===\n');

delays = distances / c; % 传播延迟 (秒)
fprintf('传播延迟范围: %.2f - %.2f μs\n', min(delays)*1e6, max(delays)*1e6);

%% 7. 信号处理流程
fprintf('=== 步骤5: 信号处理流程 ===\n');

% 准备信号处理参数
signal_params.fc = fc;
signal_params.fs = fs;
signal_params.bandwidth = bandwidth;
signal_params.oversample_factor = oversample_factor;
signal_params.rolloff_factor = rolloff_factor;
signal_params.noise_psd_dbm_hz = noise_psd_dbm_hz;
signal_params.noise_figure_db = noise_figure_db;

% 执行信号处理
[tx_signal, rx_signals, tx_baseband, rx_baseband] = signal_processing(pu_states, channel_gains, delays, signal_params);

%% 8. SNR计算
fprintf('=== 步骤6: SNR计算 ===\n');

% 计算SNR
[snr_db, signal_power_dbm, noise_power_dbm] = snr_calculation(pu_tx_power_dbm, channel_gains, signal_params);

%% 9. 结果可视化
fprintf('=== 步骤7: 结果可视化 ===\n');

% 生成所有可视化图表
visualization(tx_signal, rx_signals, tx_baseband, rx_baseband, pu_states, signal_params);

%% 10. 仿真总结
fprintf('=== 仿真总结 ===\n');
fprintf('仿真参数:\n');
fprintf('  UAV数量: %d\n', num_uav);
fprintf('  载波频率: %.1f GHz\n', fc/1e9);
fprintf('  系统带宽: %.1f MHz\n', bandwidth/1e6);
fprintf('  PU发射功率: %d dBm\n', pu_tx_power_dbm);
fprintf('  仿真时间槽: %d\n', total_time_slots);
fprintf('  符号数量: %d\n', num_symbols);

fprintf('\n关键结果:\n');
fprintf('  平均LoS概率: %.2f\n', mean(los_prob));
fprintf('  平均路径损耗: %.1f dB\n', mean(path_loss_db));
fprintf('  平均SNR: %.1f dB\n', mean(snr_db));
fprintf('  PU活动占比: %.1f%%\n', sum(pu_states)/length(pu_states)*100);
fprintf('  噪声功率: %.1f dBm\n', noise_power_dbm);

% 性能分析
good_snr_count = sum(snr_db >= 10);
fair_snr_count = sum(snr_db >= 0 & snr_db < 10);
poor_snr_count = sum(snr_db < 0);

fprintf('\nSNR性能分析:\n');
fprintf('  优秀链路 (≥10 dB): %d (%.1f%%)\n', good_snr_count, good_snr_count/num_uav*100);
fprintf('  良好链路 (0-10 dB): %d (%.1f%%)\n', fair_snr_count, fair_snr_count/num_uav*100);
fprintf('  较差链路 (<0 dB): %d (%.1f%%)\n', poor_snr_count, poor_snr_count/num_uav*100);

fprintf('\n仿真完成！所有结果已保存到图表中。\n');
fprintf('=======================================\n');