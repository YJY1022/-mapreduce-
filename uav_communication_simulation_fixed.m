%% UAV通信系统仿真 - 修复版本
% 基于LTE-5G标准的UAV与基站通信仿真
% 包含时域图、频域图和功率谱密度图的绘制

clear all; close all; clc;

%% 系统参数设置
fprintf('=== UAV通信系统仿真 ===\n');

% 基本参数 (基于图片要求)
fc = 30.72e6;           % 载波频率 30.72 MHz (LTE-5G)
T_sense = 63e-6;        % 感知周期 63 μs
N_schedule = 200;       % 感知调度周期
fs_base = 1/T_sense;    % 基础采样频率
oversample_factor = 8;  % 过采样因子
fs = fs_base * oversample_factor;  % 过采样频率

% 信号参数
signal_duration = N_schedule * T_sense;  % 总信号时长
t = 0:1/fs:signal_duration-1/fs;        % 时间向量
N = length(t);                          % 采样点数

% 打印参数信息
fprintf('系统参数:\n');
fprintf('  载波频率: %.2f MHz\n', fc/1e6);
fprintf('  感知周期: %.1f μs\n', T_sense*1e6);
fprintf('  基础采样频率: %.2f MHz\n', fs_base/1e6);
fprintf('  过采样频率: %.2f MHz\n', fs/1e6);
fprintf('  信号时长: %.2f ms\n', signal_duration*1e3);
fprintf('  采样点数: %d\n', N);

% 功率和噪声参数
P_tx = 20;              % 发射功率 (dBm)
P_tx_linear = 10^(P_tx/10) * 1e-3;  % 转换为线性单位 (W)
noise_psd = -174;       % 噪声功率谱密度 (dBm/Hz)
bandwidth = 20e6;       % 系统带宽 20 MHz
noise_power = 10^(noise_psd/10) * 1e-3 * bandwidth;  % 噪声功率

% 信道参数 (LoS/NLoS)
path_loss_los = 96;     % LoS路径损耗 (dB)
path_loss_nlos = 110;   % NLoS路径损耗 (dB)
prob_los = 0.7;         % LoS概率

%% 生成基带信号
fprintf('\n生成基带通信信号...\n');

% 生成QPSK调制信号 - 修正参数计算
data_rate = 500e3;      % 数据速率 500 kbps (降低以避免数值问题)
symbol_rate = data_rate / 2;  % QPSK符号速率
samples_per_symbol = round(fs / symbol_rate);

% 安全检查
if samples_per_symbol < 2
    samples_per_symbol = 2;
    fprintf('警告: samples_per_symbol太小，调整为2\n');
end

% 计算符号数量
num_symbols = floor(N / samples_per_symbol);
if num_symbols < 1
    num_symbols = 1;
    fprintf('警告: num_symbols太小，调整为1\n');
end

fprintf('信号参数:\n');
fprintf('  数据速率: %.1f kbps\n', data_rate/1e3);
fprintf('  符号速率: %.1f ksps\n', symbol_rate/1e3);
fprintf('  每符号采样数: %d\n', samples_per_symbol);
fprintf('  符号数量: %d\n', num_symbols);

% 生成随机数据
data_bits = randi([0 1], num_symbols * 2, 1);

% QPSK调制
I_data = 2*data_bits(1:2:end) - 1;  % {-1, +1}
Q_data = 2*data_bits(2:2:end) - 1;  % {-1, +1}
data_symbols = (I_data + 1j*Q_data) / sqrt(2);  % 归一化
data_symbols = data_symbols * sqrt(P_tx_linear);

% 上采样
upsampled_signal = upsample(data_symbols, samples_per_symbol);
signal_length = min(length(upsampled_signal), N);
baseband_signal = zeros(N, 1);
baseband_signal(1:signal_length) = upsampled_signal(1:signal_length);

% 简化的脉冲成形 (避免rcosdesign函数可能的问题)
if samples_per_symbol >= 4
    % 使用简单的低通滤波器
    lpf_order = min(20, floor(samples_per_symbol/2));
    h_lpf = fir1(lpf_order, 0.8/samples_per_symbol);
    baseband_signal = conv(baseband_signal, h_lpf, 'same');
end

%% 信道建模
fprintf('应用信道模型...\n');

% 随机选择LoS或NLoS
if rand < prob_los
    path_loss = path_loss_los;
    channel_type = 'LoS';
else
    path_loss = path_loss_nlos;
    channel_type = 'NLoS';
end

% 应用路径损耗
channel_gain = 10^(-path_loss/20);
received_signal_baseband = baseband_signal * channel_gain;

% 添加多径衰落 (Rayleigh/Rician)
if strcmp(channel_type, 'LoS')
    % Rician衰落
    K_factor = 10;  % Rician K因子 (dB)
    K_linear = 10^(K_factor/10);
    rayleigh_component = (randn(N,1) + 1j*randn(N,1)) / sqrt(2);
    los_component = sqrt(K_linear / (K_linear + 1));
    nlos_component = sqrt(1 / (K_linear + 1)) * rayleigh_component;
    fading_coeff = los_component + nlos_component;
else
    % Rayleigh衰落
    fading_coeff = (randn(N,1) + 1j*randn(N,1)) / sqrt(2);
end

received_signal_baseband = received_signal_baseband .* fading_coeff;

%% 载波调制和解调
fprintf('载波调制和信道传输...\n');

% 载波调制
carrier_cos = cos(2*pi*fc*t');
carrier_sin = sin(2*pi*fc*t');
transmitted_signal = real(received_signal_baseband) .* carrier_cos - ...
                    imag(received_signal_baseband) .* carrier_sin;

% 添加AWGN噪声
noise = sqrt(noise_power) * randn(N, 1);
received_signal = transmitted_signal + noise;

% 载波解调
demod_I = received_signal .* carrier_cos;
demod_Q = -received_signal .* carrier_sin;

% 低通滤波 (使用简单的Butterworth滤波器)
if bandwidth < fs/2
    [b, a] = butter(4, bandwidth/(fs/2));
    received_I = filter(b, a, demod_I);
    received_Q = filter(b, a, demod_Q);
    received_baseband = received_I + 1j*received_Q;
else
    received_baseband = demod_I + 1j*demod_Q;
end

%% 信号分析和绘图
fprintf('绘制分析图表...\n');

% 计算频率向量
f = (-N/2:N/2-1) * (fs/N);

% FFT计算
received_fft = fftshift(fft(received_signal));
baseband_fft = fftshift(fft(received_baseband));

% 功率谱密度计算
window_length = min(512, floor(N/4));
overlap = floor(window_length/2);
[psd_received, f_psd] = pwelch(received_signal, window_length, overlap, window_length, fs);
[psd_baseband, f_psd_bb] = pwelch(received_baseband, window_length, overlap, window_length, fs);

%% 绘图
figure('Position', [100, 100, 1400, 1000]);

% 时域图
subplot(3,2,1);
plot_samples = min(1000, N);
plot(t(1:plot_samples)*1e6, received_signal(1:plot_samples), 'b-', 'LineWidth', 1);
xlabel('时间 (μs)');
ylabel('幅度');
title('接收信号时域图 (射频)');
grid on;

subplot(3,2,2);
plot(t(1:plot_samples)*1e6, real(received_baseband(1:plot_samples)), 'r-', 'LineWidth', 1);
hold on;
plot(t(1:plot_samples)*1e6, imag(received_baseband(1:plot_samples)), 'g--', 'LineWidth', 1);
xlabel('时间 (μs)');
ylabel('幅度');
title('基带信号时域图 (I/Q)');
legend('I', 'Q');
grid on;

% 频域图
subplot(3,2,3);
plot(f/1e6, 20*log10(abs(received_fft) + eps), 'b-', 'LineWidth', 1);
xlabel('频率 (MHz)');
ylabel('幅度 (dB)');
title('接收信号频域图 (射频)');
grid on;
xlim([fc/1e6-50, fc/1e6+50]);

subplot(3,2,4);
plot(f/1e6, 20*log10(abs(baseband_fft) + eps), 'r-', 'LineWidth', 1);
xlabel('频率 (MHz)');
ylabel('幅度 (dB)');
title('基带信号频域图');
grid on;
xlim([-10, 10]);

% 功率谱密度图
subplot(3,2,5);
semilogy(f_psd/1e6, psd_received, 'b-', 'LineWidth', 1.5);
xlabel('频率 (MHz)');
ylabel('功率谱密度 (W/Hz)');
title('接收信号功率谱密度 (射频)');
grid on;

subplot(3,2,6);
semilogy(f_psd_bb/1e6, psd_baseband, 'r-', 'LineWidth', 1.5);
xlabel('频率 (MHz)');
ylabel('功率谱密度 (W/Hz)');
title('基带信号功率谱密度');
grid on;

sgtitle(sprintf('UAV通信系统仿真结果 - %s信道, 过采样因子: %d', channel_type, oversample_factor));

%% 性能分析
fprintf('\n=== 仿真结果统计 ===\n');
fprintf('信道类型: %s\n', channel_type);
fprintf('载波频率: %.2f MHz\n', fc/1e6);
fprintf('过采样频率: %.2f MHz\n', fs/1e6);
fprintf('过采样因子: %d\n', oversample_factor);
fprintf('信号时长: %.2f ms\n', signal_duration*1e3);
fprintf('采样点数: %d\n', N);
fprintf('路径损耗: %.1f dB\n', path_loss);

% 计算SNR
signal_power = var(received_baseband);
snr_db = 10*log10(signal_power / noise_power);
fprintf('估计SNR: %.2f dB\n', snr_db);

% 计算带宽利用率
spectral_efficiency = data_rate / bandwidth;
fprintf('频谱效率: %.2f bps/Hz\n', spectral_efficiency);

% 计算PAPR
papr_db = 10*log10(max(abs(received_signal).^2) / mean(abs(received_signal).^2));
fprintf('PAPR: %.2f dB\n', papr_db);

fprintf('\n仿真完成！\n');

%% 保存结果
save('uav_simulation_results_fixed.mat', 't', 'received_signal', 'received_baseband', ...
     'f', 'received_fft', 'baseband_fft', 'psd_received', 'psd_baseband', ...
     'f_psd', 'f_psd_bb', 'channel_type', 'snr_db');

fprintf('结果已保存到 uav_simulation_results_fixed.mat\n');