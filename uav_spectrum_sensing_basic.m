%% 无人机协作频谱感知仿真 (基础版本)
% 基于图片中的参数要求，实现过采样条件下的频谱感知
% 不依赖信号处理工具箱，使用基础MATLAB函数
% Author: AI Assistant
% Date: 2024

clear all; close all; clc;

%% 仿真参数配置
% 基本参数（根据图片中的要求）
fc = 30.72e6;           % 载波频率 30.72 MHz (LTE 5G载波)
sensing_period = 6.5e-6; % 感知周期 6.5 μs
N_oversample = 200;     % 过采样率 N=200
fs_base = 1/sensing_period; % 基础采样频率
fs = fs_base * N_oversample; % 过采样频率

% 仿真参数
T_sim = 50e-6;          % 仿真时间长度 50 μs (减少以节省内存)
t = 0:1/fs:T_sim-1/fs;  % 时间向量
SNR_dB = [0, 5, 10, 15, 20]; % 信噪比范围
num_uavs = 4;           % 无人机数量

% 信号参数
signal_bw = 5e6;        % 信号带宽 5 MHz
psd_noise = -174;       % 噪声功率谱密度 (dBm/Hz)

fprintf('仿真参数配置完成:\n');
fprintf('载波频率: %.2f MHz\n', fc/1e6);
fprintf('过采样频率: %.2f MHz\n', fs/1e6);
fprintf('感知周期: %.1f μs\n', sensing_period*1e6);
fprintf('过采样率: %d\n', N_oversample);

%% 信号生成
% 生成主用户信号（存在概率为0.5）
pu_present = rand > 0.5;  % 随机决定主用户是否存在
signal_power_dbm = -50;   % 信号功率 -50 dBm
signal_power_linear = 10^((signal_power_dbm-30)/10); % 转换为线性功率

if pu_present
    % 生成简单的正弦波信号作为主用户信号
    f_signal = fc + 1e6;  % 信号频率稍偏离载波
    pu_signal = sqrt(signal_power_linear) * sin(2*pi*f_signal*t);
    
    % 添加少量相位调制模拟数字调制
    phase_mod = 0.5 * sin(2*pi*1e5*t); % 100kHz的相位调制
    pu_signal = sqrt(signal_power_linear) * sin(2*pi*f_signal*t + phase_mod);
    
    fprintf('主用户信号已生成 (功率: %.1f dBm)\n', signal_power_dbm);
else
    pu_signal = zeros(size(t));
    fprintf('主用户信号不存在\n');
end

%% 多无人机接收信号生成
uav_signals = cell(num_uavs, length(SNR_dB));
noise_powers = zeros(1, length(SNR_dB));

for snr_idx = 1:length(SNR_dB)
    snr = SNR_dB(snr_idx);
    
    % 计算噪声功率
    if pu_present
        signal_avg_power = mean(pu_signal.^2);
        noise_power = signal_avg_power / (10^(snr/10));
    else
        % 当没有信号时，设置固定的噪声功率
        noise_power = 10^((psd_noise-30)/10) * fs;
    end
    noise_powers(snr_idx) = noise_power;
    
    for uav_idx = 1:num_uavs
        % 生成高斯白噪声
        noise = sqrt(noise_power) * randn(size(t));
        
        % 模拟不同无人机的信道衰落（简化模型）
        channel_gain = 0.8 + 0.4*rand(); % 随机信道增益
        delay_samples = round(rand() * fs * 1e-6); % 随机路径延迟（最大1μs）
        
        % 生成接收信号
        if delay_samples > 0 && delay_samples < length(pu_signal)
            delayed_signal = [zeros(1, delay_samples), pu_signal(1:end-delay_samples)];
        else
            delayed_signal = pu_signal;
        end
        
        received_signal = channel_gain * delayed_signal + noise;
        uav_signals{uav_idx, snr_idx} = received_signal;
    end
    
    fprintf('SNR = %d dB 条件下的信号已生成\n', snr);
end

%% 频谱感知算法实现
% 使用能量检测算法
detection_results = zeros(num_uavs, length(SNR_dB));
energy_levels = zeros(num_uavs, length(SNR_dB));

for snr_idx = 1:length(SNR_dB)
    for uav_idx = 1:num_uavs
        signal = uav_signals{uav_idx, snr_idx};
        
        % 能量检测
        signal_energy = sum(signal.^2) / length(signal);
        energy_levels(uav_idx, snr_idx) = signal_energy;
        
        % 简单阈值检测（基于噪声功率）
        threshold = 2 * noise_powers(snr_idx); % 阈值设为噪声功率的2倍
        detection_results(uav_idx, snr_idx) = signal_energy > threshold;
    end
end

%% 协作决策
% 使用多数投票法进行协作决策
cooperative_decisions = zeros(1, length(SNR_dB));
for snr_idx = 1:length(SNR_dB)
    votes = sum(detection_results(:, snr_idx));
    cooperative_decisions(snr_idx) = votes > num_uavs/2;
end

fprintf('\n协作频谱感知结果:\n');
for snr_idx = 1:length(SNR_dB)
    fprintf('SNR = %d dB: 检测到主用户 = %s\n', SNR_dB(snr_idx), ...
        num2str(cooperative_decisions(snr_idx)));
end

%% 绘图部分
% 选择中等SNR (10 dB) 的第一个无人机信号进行可视化
plot_snr_idx = find(SNR_dB == 10);
if isempty(plot_snr_idx)
    plot_snr_idx = ceil(length(SNR_dB)/2);
end
plot_signal = uav_signals{1, plot_snr_idx};

% 创建图形窗口
figure('Position', [100, 100, 1200, 800]);

% 1. 时域图
subplot(2,3,1);
t_us = t * 1e6; % 转换为微秒
plot(t_us, plot_signal, 'b-', 'LineWidth', 1);
xlabel('时间 (μs)');
ylabel('幅度');
title(sprintf('接收信号时域图 (SNR = %d dB)', SNR_dB(plot_snr_idx)));
grid on;
xlim([0, min(10, max(t_us))]); % 显示前10μs或全部

% 2. 频域图 (FFT) - 使用基础FFT
subplot(2,3,2);
N_fft = min(1024, length(plot_signal)); % 适应信号长度
signal_fft = fft(plot_signal(1:N_fft));
signal_fft_shifted = fftshift(signal_fft);
f = (-N_fft/2:N_fft/2-1) * fs / N_fft / 1e6; % 频率轴 (MHz)
plot(f, abs(signal_fft_shifted), 'r-', 'LineWidth', 1);
xlabel('频率 (MHz)');
ylabel('幅度');
title('接收信号频域图 (FFT)');
grid on;

% 3. 简化的功率谱密度图
subplot(2,3,3);
% 使用简单的周期图方法估计PSD
psd_simple = abs(signal_fft_shifted).^2 / (N_fft * fs);
plot(f, 10*log10(psd_simple + eps), 'g-', 'LineWidth', 1);
xlabel('频率 (MHz)');
ylabel('功率谱密度 (dB/Hz)');
title('功率谱密度图 (周期图法)');
grid on;

% 4. 能量检测结果
subplot(2,3,4);
bar(SNR_dB, squeeze(energy_levels(1, :)));
xlabel('信噪比 (dB)');
ylabel('能量级别');
title('无人机1的能量检测结果');
grid on;

% 5. 检测概率比较
subplot(2,3,5);
detection_prob = mean(detection_results, 1); % 各SNR下的平均检测概率
plot(SNR_dB, detection_prob, 'mo-', 'LineWidth', 2, 'MarkerSize', 8);
hold on;
plot(SNR_dB, cooperative_decisions, 'bs-', 'LineWidth', 2, 'MarkerSize', 8);
xlabel('信噪比 (dB)');
ylabel('检测概率');
title('频谱感知性能');
legend('个体检测', '协作检测', 'Location', 'best');
grid on;
ylim([0, 1.1]);

% 6. 过采样效果展示
subplot(2,3,6);
% 比较过采样前后的频谱
signal_downsample = plot_signal(1:N_oversample:end); % 下采样
fs_down = fs / N_oversample;

% 原信号频谱
N_orig = min(512, length(plot_signal));
fft_orig = fftshift(fft(plot_signal(1:N_orig)));
f_orig = (-N_orig/2:N_orig/2-1) * fs / N_orig / 1e6;

% 下采样信号频谱
N_down = min(64, length(signal_downsample));
fft_down = fftshift(fft(signal_downsample(1:N_down)));
f_down = (-N_down/2:N_down/2-1) * fs_down / N_down / 1e6;

plot(f_orig, 10*log10(abs(fft_orig).^2 + eps), 'b-', 'LineWidth', 1.5);
hold on;
plot(f_down, 10*log10(abs(fft_down).^2 + eps), 'r--', 'LineWidth', 1.5);
xlabel('频率 (MHz)');
ylabel('功率 (dB)');
title('过采样效果比较');
legend('过采样', '正常采样', 'Location', 'best');
grid on;

% 设置总标题
if pu_present
    sgtitle('无人机协作频谱感知仿真结果 (主用户存在)', 'FontSize', 14, 'FontWeight', 'bold');
else
    sgtitle('无人机协作频谱感知仿真结果 (主用户不存在)', 'FontSize', 14, 'FontWeight', 'bold');
end

%% 性能评估
fprintf('\n=== 性能评估 ===\n');
fprintf('过采样频率: %.2f MHz\n', fs/1e6);
fprintf('频率分辨率: %.2f kHz\n', fs/N_fft/1e3);
if pu_present
    fprintf('主用户状态: 存在\n');
else
    fprintf('主用户状态: 不存在\n');
end

if pu_present
    true_positive_rate = mean(cooperative_decisions);
    fprintf('协作检测准确率: %.2f%%\n', true_positive_rate * 100);
else
    false_alarm_rate = mean(cooperative_decisions);
    fprintf('虚警率: %.2f%%\n', false_alarm_rate * 100);
end

fprintf('仿真完成!\n');

%% 额外的性能分析
fprintf('\n=== 详细分析 ===\n');
fprintf('各无人机检测结果:\n');
for uav_idx = 1:num_uavs
    fprintf('无人机 %d: ', uav_idx);
    for snr_idx = 1:length(SNR_dB)
        if detection_results(uav_idx, snr_idx)
            fprintf('[%d dB: 检测] ', SNR_dB(snr_idx));
        else
            fprintf('[%d dB: 未检测] ', SNR_dB(snr_idx));
        end
    end
    fprintf('\n');
end