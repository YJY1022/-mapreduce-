%% UAV通信系统仿真
% 基于图片中的参数配置，实现UAV通信链路仿真
% 包括成型滤波、上变频、信道建模、下变频、匹配滤波等完整信号处理链路

clear all; close all; clc;

%% 系统参数配置（可修改）
% 基本通信参数
fc = 30.72e9;           % 载波频率 30.72 GHz (毫米波)
fs = 4*fc;              % 采样频率（过采样）
symbol_rate = 1e6;      % 符号速率 1 Msps
oversample = fs/symbol_rate; % 过采样倍数

% UAV参数（基于图片中的配置）
uav_height = 100;       % UAV高度 (m)
uav_speed = 50;         % UAV速度 (m/s) 
distance_2d = 500;      % 水平距离 (m)
elevation_angle = atan(uav_height/distance_2d); % 仰角

% 信道参数
SNR_dB = 20;            % 信噪比 (dB)
doppler_freq = fc * uav_speed * cos(elevation_angle) / 3e8; % 多普勒频移

% LoS信道参数（基于图片）
A = 9.6;                % LoS路径损耗参数
B = 0.16;               % 环境因子
path_loss_exp = 2;      % 路径损耗指数

% 信号参数
num_symbols = 1000;     % 符号数
bits_per_symbol = 2;    % QPSK调制

%% 生成基带信号
% 生成随机比特
data_bits = randi([0,1], num_symbols*bits_per_symbol, 1);

% QPSK调制
qpsk_symbols = [];
for i = 1:2:length(data_bits)
    if data_bits(i) == 0 && data_bits(i+1) == 0
        qpsk_symbols = [qpsk_symbols; 1+1j];
    elseif data_bits(i) == 0 && data_bits(i+1) == 1
        qpsk_symbols = [qpsk_symbols; -1+1j];
    elseif data_bits(i) == 1 && data_bits(i+1) == 0
        qpsk_symbols = [qpsk_symbols; 1-1j];
    else
        qpsk_symbols = [qpsk_symbols; -1-1j];
    end
end
qpsk_symbols = qpsk_symbols / sqrt(2); % 归一化

%% 成型滤波（发射滤波器）
% 根升余弦滤波器
rolloff = 0.25;         % 滚降因子
filter_span = 8;        % 滤波器长度（符号周期）
sps = round(oversample/symbol_rate*fs); % 每符号采样点数

% 设计根升余弦滤波器
rrc_filter = rcosdesign(rolloff, filter_span, sps, 'sqrt');

% 上采样并成型滤波
upsampled_signal = upsample(qpsk_symbols, sps);
shaped_signal = conv(upsampled_signal, rrc_filter, 'same');

%% 上变频到载波频率
t = (0:length(shaped_signal)-1) / fs;
carrier = exp(1j*2*pi*fc*t');
tx_signal = real(shaped_signal .* carrier);

%% 信道建模
% LoS路径损耗计算
distance_3d = sqrt(distance_2d^2 + uav_height^2);
path_loss_dB = A * log10(distance_3d) + B + 20*log10(fc/1e9); % 自由空间路径损耗

% 信道增益
channel_gain = 10^(-path_loss_dB/20);

% 多普勒效应（频率偏移）
doppler_phase = 2*pi*doppler_freq*t';
doppler_effect = exp(1j*doppler_phase);

% 应用信道效应
channel_signal = tx_signal * channel_gain;

% 添加高斯白噪声
signal_power = mean(abs(channel_signal).^2);
noise_power = signal_power / (10^(SNR_dB/10));
noise = sqrt(noise_power/2) * (randn(size(channel_signal)) + 1j*randn(size(channel_signal)));
rx_signal = channel_signal + real(noise);

%% 下变频
local_oscillator = exp(-1j*2*pi*fc*t');
baseband_signal = rx_signal .* local_oscillator;

% 低通滤波去除高频分量
cutoff_freq = symbol_rate * 2; % 截止频率
[b,a] = butter(6, cutoff_freq/(fs/2));
filtered_signal = filter(b, a, baseband_signal);

%% 匹配滤波
matched_filtered = conv(filtered_signal, conj(fliplr(rrc_filter')), 'same');

% 下采样到符号速率
decimated_signal = downsample(matched_filtered, sps);

%% 时域分析
figure('Position', [100, 100, 1200, 800]);

subplot(3,3,1);
plot(real(shaped_signal(1:1000)));
title('发射基带信号（成型后）');
xlabel('采样点'); ylabel('幅度');
grid on;

subplot(3,3,2);
plot(tx_signal(1:1000));
title('发射射频信号');
xlabel('采样点'); ylabel('幅度');
grid on;

subplot(3,3,3);
plot(rx_signal(1:1000));
title('接收射频信号（含噪声）');
xlabel('采样点'); ylabel('幅度');
grid on;

subplot(3,3,4);
plot(real(baseband_signal(1:1000)));
title('下变频后基带信号');
xlabel('采样点'); ylabel('幅度');
grid on;

subplot(3,3,5);
plot(real(matched_filtered(1:1000)));
title('匹配滤波后信号');
xlabel('采样点'); ylabel('幅度');
grid on;

%% 频域分析
N = length(tx_signal);
f = (-N/2:N/2-1) * fs/N;

% 发射信号频谱
TX_spectrum = fftshift(fft(tx_signal));
subplot(3,3,6);
plot(f/1e9, 20*log10(abs(TX_spectrum)));
title('发射信号频谱');
xlabel('频率 (GHz)'); ylabel('幅度 (dB)');
grid on;

% 接收信号频谱
RX_spectrum = fftshift(fft(rx_signal));
subplot(3,3,7);
plot(f/1e9, 20*log10(abs(RX_spectrum)));
title('接收信号频谱');
xlabel('频率 (GHz)'); ylabel('幅度 (dB)');
grid on;

%% 功率谱密度分析
% 使用Welch方法估计功率谱密度
window_length = min(1024, floor(length(rx_signal)/4));
[psd_tx, f_psd] = pwelch(tx_signal, window_length, [], [], fs);
[psd_rx, ~] = pwelch(rx_signal, window_length, [], [], fs);

subplot(3,3,8);
semilogy(f_psd/1e9, psd_tx);
title('发射信号功率谱密度');
xlabel('频率 (GHz)'); ylabel('PSD (W/Hz)');
grid on;

subplot(3,3,9);
semilogy(f_psd/1e9, psd_rx);
title('接收信号功率谱密度');
xlabel('频率 (GHz)'); ylabel('PSD (W/Hz)');
grid on;

%% 性能分析
fprintf('=== UAV通信系统仿真结果 ===\n');
fprintf('载波频率: %.2f GHz\n', fc/1e9);
fprintf('UAV高度: %d m\n', uav_height);
fprintf('水平距离: %d m\n', distance_2d);
fprintf('3D距离: %.2f m\n', distance_3d);
fprintf('仰角: %.2f 度\n', elevation_angle*180/pi);
fprintf('路径损耗: %.2f dB\n', path_loss_dB);
fprintf('多普勒频移: %.2f Hz\n', doppler_freq);
fprintf('信噪比: %d dB\n', SNR_dB);
fprintf('符号速率: %.2f Msps\n', symbol_rate/1e6);
fprintf('过采样倍数: %.0f\n', sps);

%% 星座图分析
figure('Position', [1350, 100, 800, 600]);

subplot(2,2,1);
scatter(real(qpsk_symbols), imag(qpsk_symbols), 'bo', 'filled');
title('发射星座图');
xlabel('I'); ylabel('Q');
grid on; axis equal;

subplot(2,2,2);
scatter(real(decimated_signal), imag(decimated_signal), 'ro', 'MarkerSize', 3);
title('接收星座图（匹配滤波后）');
xlabel('I'); ylabel('Q');
grid on; axis equal;

% EVM计算
ideal_symbols = repmat(qpsk_symbols, 1, 1);
received_symbols = decimated_signal(1:length(qpsk_symbols));
evm_rms = sqrt(mean(abs(received_symbols - ideal_symbols).^2)) / sqrt(mean(abs(ideal_symbols).^2));
evm_percent = evm_rms * 100;

subplot(2,2,3);
plot(1:length(qpsk_symbols), abs(received_symbols - ideal_symbols));
title('误差向量幅度 (EVM)');
xlabel('符号索引'); ylabel('EVM');
grid on;

subplot(2,2,4);
% 信道冲击响应
h_channel = [channel_gain]; % 简化的信道冲击响应
stem(0, abs(h_channel), 'filled');
title('信道冲击响应');
xlabel('时延'); ylabel('幅度');
grid on;

fprintf('EVM: %.2f%%\n', evm_percent);

%% 保存仿真参数
sim_params = struct();
sim_params.fc = fc;
sim_params.fs = fs;
sim_params.symbol_rate = symbol_rate;
sim_params.uav_height = uav_height;
sim_params.distance_2d = distance_2d;
sim_params.SNR_dB = SNR_dB;
sim_params.path_loss_dB = path_loss_dB;
sim_params.doppler_freq = doppler_freq;
sim_params.evm_percent = evm_percent;

save('uav_simulation_results.mat', 'sim_params', 'tx_signal', 'rx_signal', ...
     'shaped_signal', 'matched_filtered', 'qpsk_symbols', 'decimated_signal');

disp('仿真完成！结果已保存到 uav_simulation_results.mat');