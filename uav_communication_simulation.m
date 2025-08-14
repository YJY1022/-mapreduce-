%% UAV通信系统仿真 - 过采样条件下的信号分析
% 基于LTE-5G标准的UAV与基站通信仿真
% 包含时域图、频域图和功率谱密度图的绘制

clear all; close all; clc;

%% 系统参数设置
% 基本参数
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
fprintf('生成基带通信信号...\n');

% 生成QPSK调制信号
data_rate = 1e6;        % 数据速率 1 Mbps
symbol_rate = data_rate / 2;  % QPSK符号速率
samples_per_symbol = round(fs / symbol_rate);

% 检查关键参数的有效性
fprintf('调试信息:\n');
fprintf('  采样频率 fs = %.2e Hz\n', fs);
fprintf('  符号速率 symbol_rate = %.2e symbols/s\n', symbol_rate);
fprintf('  每符号采样数 samples_per_symbol = %d\n', samples_per_symbol);
fprintf('  总采样点数 N = %d\n', N);

% 确保samples_per_symbol是正整数
if isnan(samples_per_symbol) || isinf(samples_per_symbol) || samples_per_symbol <= 0
    error('samples_per_symbol计算错误: fs=%.2e, symbol_rate=%.2e', fs, symbol_rate);
end
samples_per_symbol = max(1, round(samples_per_symbol));

% 生成随机数据
num_symbols = floor(N / samples_per_symbol);

% 检查num_symbols的有效性
if isnan(num_symbols) || isinf(num_symbols) || num_symbols <= 0
    error('num_symbols计算错误: N=%d, samples_per_symbol=%d, num_symbols=%d', N, samples_per_symbol, num_symbols);
end

% 确保num_symbols是正整数
num_symbols = max(1, floor(num_symbols));
data_bits = randi([0 1], num_symbols * 2, 1);

% QPSK调制
data_symbols = (data_bits(1:2:end) - 0.5) + 1j*(data_bits(2:2:end) - 0.5);
data_symbols = data_symbols * sqrt(P_tx_linear);

% 上采样和脉冲成形
upsampled_signal = upsample(data_symbols, samples_per_symbol);
if length(upsampled_signal) > N
    upsampled_signal = upsampled_signal(1:N);
else
    upsampled_signal = [upsampled_signal; zeros(N-length(upsampled_signal), 1)];
end

% 根升余弦滤波器脉冲成形
rolloff = 0.25;
span = 6;
sps = samples_per_symbol;
rrcfilter = rcosdesign(rolloff, span, sps);
baseband_signal = conv(upsampled_signal, rrcfilter, 'same');

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
carrier = exp(1j*2*pi*fc*t');
transmitted_signal = real(received_signal_baseband .* carrier);

% 添加AWGN噪声
noise = sqrt(noise_power/2) * (randn(N,1) + 1j*randn(N,1));
received_signal = transmitted_signal + real(noise);

% 载波解调
demodulated_signal = received_signal .* cos(2*pi*fc*t');
% 低通滤波
[b, a] = butter(6, bandwidth/(fs/2));
received_baseband = filter(b, a, demodulated_signal);

%% 信号分析和绘图
fprintf('绘制分析图表...\n');

% 计算频率向量
f = (-N/2:N/2-1) * (fs/N);
f_positive = (0:N/2-1) * (fs/N);

% FFT计算
received_fft = fftshift(fft(received_signal));
baseband_fft = fftshift(fft(received_baseband));

% 功率谱密度计算
[psd_received, f_psd] = pwelch(received_signal, hamming(1024), 512, 1024, fs);
[psd_baseband, f_psd_bb] = pwelch(received_baseband, hamming(1024), 512, 1024, fs);

%% 绘图
figure('Position', [100, 100, 1400, 1000]);

% 时域图
subplot(3,2,1);
plot(t(1:1000)*1e6, real(received_signal(1:1000)), 'b-', 'LineWidth', 1);
xlabel('时间 (μs)');
ylabel('幅度');
title('接收信号时域图 (射频)');
grid on;

subplot(3,2,2);
plot(t(1:1000)*1e6, real(received_baseband(1:1000)), 'r-', 'LineWidth', 1);
xlabel('时间 (μs)');
ylabel('幅度');
title('基带信号时域图');
grid on;

% 频域图
subplot(3,2,3);
plot(f/1e6, 20*log10(abs(received_fft)), 'b-', 'LineWidth', 1);
xlabel('频率 (MHz)');
ylabel('幅度 (dB)');
title('接收信号频域图 (射频)');
grid on;
xlim([-fs/2e6, fs/2e6]);

subplot(3,2,4);
plot(f/1e6, 20*log10(abs(baseband_fft)), 'r-', 'LineWidth', 1);
xlabel('频率 (MHz)');
ylabel('幅度 (dB)');
title('基带信号频域图');
grid on;
xlim([-fs/2e6, fs/2e6]);

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
signal_power = var(real(received_baseband));
estimated_noise_power = var(received_signal) - var(transmitted_signal);
snr_db = 10*log10(signal_power / estimated_noise_power);
fprintf('估计SNR: %.2f dB\n', snr_db);

% 计算带宽利用率
occupied_bandwidth = bandwidth;
spectral_efficiency = data_rate / occupied_bandwidth;
fprintf('频谱效率: %.2f bps/Hz\n', spectral_efficiency);

fprintf('\n仿真完成！\n');

%% 保存结果
save('uav_simulation_results.mat', 't', 'received_signal', 'received_baseband', ...
     'f', 'received_fft', 'baseband_fft', 'psd_received', 'psd_baseband', ...
     'f_psd', 'f_psd_bb', 'channel_type', 'snr_db');

fprintf('结果已保存到 uav_simulation_results.mat\n');