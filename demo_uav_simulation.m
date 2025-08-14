%% UAV通信系统仿真演示脚本
% 快速演示过采样条件下的信号分析
% 简化版本，便于理解和验证

clear all; close all; clc;

fprintf('=== UAV通信系统仿真演示 ===\n');

%% 基本参数 (基于图片要求)
fc = 30.72e6;           % 载波频率 30.72 MHz (LTE-5G标准)
T_sense = 63e-6;        % 感知周期 63 μs
N_schedule = 50;        % 简化的调度周期 (减少计算时间)
oversample_factor = 4;  % 过采样因子

% 计算采样参数
fs_base = 1/T_sense;
fs = fs_base * oversample_factor;
signal_duration = N_schedule * T_sense;
t = 0:1/fs:signal_duration-1/fs;
N = length(t);

fprintf('系统参数:\n');
fprintf('  载波频率: %.2f MHz\n', fc/1e6);
fprintf('  感知周期: %.1f μs\n', T_sense*1e6);
fprintf('  基础采样频率: %.2f MHz\n', fs_base/1e6);
fprintf('  过采样频率: %.2f MHz\n', fs/1e6);
fprintf('  信号时长: %.2f ms\n', signal_duration*1e3);
fprintf('  采样点数: %d\n', N);

%% 生成基带QPSK信号
fprintf('\n生成QPSK调制信号...\n');

data_rate = 500e3;      % 数据速率 500 kbps (简化)
symbol_rate = data_rate / 2;
samples_per_symbol = round(fs / symbol_rate);

% 生成随机数据
num_symbols = floor(N / samples_per_symbol);
data_bits = randi([0 1], num_symbols * 2, 1);

% QPSK调制
I_data = 2*data_bits(1:2:end) - 1;
Q_data = 2*data_bits(2:2:end) - 1;
complex_symbols = (I_data + 1j*Q_data) / sqrt(2);

% 上采样
upsampled_signal = upsample(complex_symbols, samples_per_symbol);
if length(upsampled_signal) > N
    upsampled_signal = upsampled_signal(1:N);
else
    upsampled_signal = [upsampled_signal; zeros(N-length(upsampled_signal), 1)];
end

% 简化的脉冲成形 (矩形脉冲)
baseband_signal = upsampled_signal;

%% 信道模型
fprintf('应用信道模型...\n');

% 功率设置
P_tx_dbm = 20;          % 发射功率 20 dBm
P_tx = 10^(P_tx_dbm/10) * 1e-3;  % 转换为瓦特
baseband_signal = baseband_signal * sqrt(P_tx);

% LoS信道 (简化)
path_loss_db = 96;      % 路径损耗
channel_gain = 10^(-path_loss_db/20);

% Rician衰落 (LoS条件)
K_factor_db = 10;       % Rician K因子
K = 10^(K_factor_db/10);
h_los = sqrt(K/(K+1));  % LoS分量
h_nlos = sqrt(1/(K+1)) * (randn(N,1) + 1j*randn(N,1))/sqrt(2); % 散射分量
channel_coeff = h_los + h_nlos;

% 应用信道
received_baseband = baseband_signal .* channel_coeff * channel_gain;

%% 载波调制
fprintf('载波调制...\n');

% 生成载波
carrier_cos = cos(2*pi*fc*t');
carrier_sin = sin(2*pi*fc*t');

% 调制到射频
rf_signal = real(received_baseband) .* carrier_cos - imag(received_baseband) .* carrier_sin;

%% 添加噪声
fprintf('添加AWGN噪声...\n');

% 噪声参数
noise_psd_dbm = -174;   % 噪声功率谱密度 -174 dBm/Hz
bandwidth = 20e6;       % 系统带宽 20 MHz
noise_power = 10^(noise_psd_dbm/10) * 1e-3 * bandwidth;

% 添加白噪声
noise = sqrt(noise_power) * randn(N, 1);
received_signal = rf_signal + noise;

%% 载波解调
fprintf('载波解调...\n');

% 同步解调
demod_I = received_signal .* carrier_cos;
demod_Q = -received_signal .* carrier_sin;

% 低通滤波 (简化的移动平均)
filter_length = round(samples_per_symbol/2);
lpf_I = conv(demod_I, ones(filter_length,1)/filter_length, 'same');
lpf_Q = conv(demod_Q, ones(filter_length,1)/filter_length, 'same');
received_baseband_demod = lpf_I + 1j*lpf_Q;

%% 信号分析
fprintf('分析信号特性...\n');

% 计算SNR
signal_power = var(received_baseband);
snr_db = 10*log10(signal_power / noise_power);

% 计算频率向量
f = (-N/2:N/2-1) * (fs/N);

% FFT分析
rf_fft = fftshift(fft(received_signal));
baseband_fft = fftshift(fft(received_baseband_demod));

% 功率谱密度
window_length = min(1024, floor(N/4));
[psd_rf, f_psd] = pwelch(received_signal, hamming(window_length), [], [], fs);
[psd_bb, f_psd_bb] = pwelch(received_baseband_demod, hamming(window_length), [], [], fs);

%% 绘制结果
fprintf('绘制分析图表...\n');

figure('Position', [50, 50, 1600, 1000]);

% 时域图
subplot(3,3,1);
plot_samples = min(500, N);
plot(t(1:plot_samples)*1e6, real(received_signal(1:plot_samples)), 'b-', 'LineWidth', 1);
xlabel('时间 (μs)');
ylabel('幅度');
title('接收信号时域图 (射频)');
grid on;

subplot(3,3,2);
plot(t(1:plot_samples)*1e6, real(received_baseband_demod(1:plot_samples)), 'r-', 'LineWidth', 1);
hold on;
plot(t(1:plot_samples)*1e6, imag(received_baseband_demod(1:plot_samples)), 'g--', 'LineWidth', 1);
xlabel('时间 (μs)');
ylabel('幅度');
title('基带信号时域图 (I/Q)');
legend('I', 'Q');
grid on;

subplot(3,3,3);
plot(real(received_baseband_demod(1:plot_samples)), imag(received_baseband_demod(1:plot_samples)), 'ro', 'MarkerSize', 3);
xlabel('同相分量 (I)');
ylabel('正交分量 (Q)');
title('星座图');
grid on;
axis equal;

% 频域图
subplot(3,3,4);
plot(f/1e6, 20*log10(abs(rf_fft)), 'b-', 'LineWidth', 1);
xlabel('频率 (MHz)');
ylabel('幅度 (dB)');
title('射频信号频谱');
grid on;
xlim([fc/1e6-50, fc/1e6+50]);

subplot(3,3,5);
plot(f/1e6, 20*log10(abs(baseband_fft)), 'r-', 'LineWidth', 1);
xlabel('频率 (MHz)');
ylabel('幅度 (dB)');
title('基带信号频谱');
grid on;
xlim([-10, 10]);

subplot(3,3,6);
semilogy(f_psd/1e6, psd_rf, 'b-', 'LineWidth', 1);
xlabel('频率 (MHz)');
ylabel('功率谱密度 (W/Hz)');
title('射频信号PSD');
grid on;

% 性能分析图
subplot(3,3,7);
stem(1:min(20, num_symbols), real(complex_symbols(1:min(20, num_symbols))), 'b', 'filled');
hold on;
stem(1:min(20, num_symbols), imag(complex_symbols(1:min(20, num_symbols))), 'r', 'filled');
xlabel('符号索引');
ylabel('符号值');
title('发送符号序列');
legend('I', 'Q');
grid on;

subplot(3,3,8);
% 解调符号 (简化采样)
sample_points = samples_per_symbol:samples_per_symbol:length(received_baseband_demod);
sample_points = sample_points(sample_points <= length(received_baseband_demod));
sampled_symbols = received_baseband_demod(sample_points);
plot(real(sampled_symbols), imag(sampled_symbols), 'go', 'MarkerSize', 4, 'MarkerFaceColor', 'g');
hold on;
plot([-1, 1, 1, -1, -1]/sqrt(2), [-1, -1, 1, 1, -1]/sqrt(2), 'k--', 'LineWidth', 1);
xlabel('同相分量');
ylabel('正交分量');
title('接收符号星座图');
grid on;
axis equal;

subplot(3,3,9);
% 误差向量幅度
if length(sampled_symbols) >= length(complex_symbols)
    error_vector = sampled_symbols(1:length(complex_symbols)) - complex_symbols;
else
    error_vector = sampled_symbols - complex_symbols(1:length(sampled_symbols));
end
evm = abs(error_vector);
plot(evm, 'mo-', 'LineWidth', 1, 'MarkerSize', 3);
xlabel('符号索引');
ylabel('EVM');
title('误差向量幅度');
grid on;

sgtitle(sprintf('UAV通信系统仿真结果 - 过采样因子: %d, SNR: %.1f dB', oversample_factor, snr_db));

%% 结果统计
fprintf('\n=== 仿真结果统计 ===\n');
fprintf('过采样因子: %d\n', oversample_factor);
fprintf('估计SNR: %.2f dB\n', snr_db);
fprintf('符号速率: %.1f ksps\n', symbol_rate/1e3);
fprintf('频谱效率: %.2f bps/Hz\n', data_rate/bandwidth);

% 计算EVM
mean_evm = mean(abs(error_vector)) / mean(abs(complex_symbols)) * 100;
fprintf('平均EVM: %.2f%%\n', mean_evm);

% 计算PAPR
papr_db = 10*log10(max(abs(received_signal).^2) / mean(abs(received_signal).^2));
fprintf('PAPR: %.2f dB\n', papr_db);

fprintf('\n演示完成！\n');

%% 保存结果
save('demo_results.mat', 't', 'received_signal', 'received_baseband_demod', ...
     'rf_fft', 'baseband_fft', 'psd_rf', 'psd_bb', 'snr_db', 'mean_evm');
fprintf('结果已保存到 demo_results.mat\n');