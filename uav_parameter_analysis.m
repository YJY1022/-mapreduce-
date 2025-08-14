%% UAV通信系统参数分析脚本
% 比较不同过采样因子和信道条件的性能
% 生成详细的性能分析报告

clear all; close all; clc;

%% 参数设置
fprintf('=== UAV通信系统参数分析 ===\n');

% 基本系统参数 (基于图片要求)
fc = 30.72e6;           % 载波频率 30.72 MHz
T_sense = 63e-6;        % 感知周期 63 μs  
N_schedule = 200;       % 感知调度周期
fs_base = 1/T_sense;    % 基础采样频率

% 测试不同的过采样因子
oversample_factors = [2, 4, 8, 16];
channel_types = {'LoS', 'NLoS'};

% 存储结果
results = struct();

%% 遍历不同参数组合
for ch_idx = 1:length(channel_types)
    channel_type = channel_types{ch_idx};
    
    for os_idx = 1:length(oversample_factors)
        oversample_factor = oversample_factors(os_idx);
        
        fprintf('\n分析: %s信道, 过采样因子 %d\n', channel_type, oversample_factor);
        
        % 计算采样参数
        fs = fs_base * oversample_factor;
        signal_duration = N_schedule * T_sense;
        t = 0:1/fs:signal_duration-1/fs;
        N = length(t);
        
        % 生成测试信号
        [received_signal, received_baseband, params] = generate_uav_signal(fc, fs, N, channel_type);
        
        % 分析信号
        analysis = analyze_signal_performance(received_signal, received_baseband, fs, params);
        
        % 存储结果
        key = sprintf('%s_OS%d', channel_type, oversample_factor);
        results.(key) = analysis;
        results.(key).oversample_factor = oversample_factor;
        results.(key).channel_type = channel_type;
    end
end

%% 绘制比较图
plot_performance_comparison(results, oversample_factors, channel_types);

%% 生成报告
generate_performance_report(results, oversample_factors, channel_types);

fprintf('\n参数分析完成！\n');

%% 辅助函数

function [received_signal, received_baseband, params] = generate_uav_signal(fc, fs, N, channel_type)
    % 生成UAV通信信号
    
    % 信号参数
    data_rate = 1e6;
    symbol_rate = data_rate / 2;
    samples_per_symbol = round(fs / symbol_rate);
    bandwidth = 20e6;
    
    % 功率参数
    P_tx = 20; % dBm
    P_tx_linear = 10^(P_tx/10) * 1e-3;
    noise_psd = -174; % dBm/Hz
    noise_power = 10^(noise_psd/10) * 1e-3 * bandwidth;
    
    % 生成QPSK调制信号
    num_symbols = floor(N / samples_per_symbol);
    data_bits = randi([0 1], num_symbols * 2, 1);
    data_symbols = (data_bits(1:2:end) - 0.5) + 1j*(data_bits(2:2:end) - 0.5);
    data_symbols = data_symbols * sqrt(P_tx_linear);
    
    % 上采样和脉冲成形
    upsampled_signal = upsample(data_symbols, samples_per_symbol);
    if length(upsampled_signal) > N
        upsampled_signal = upsampled_signal(1:N);
    else
        upsampled_signal = [upsampled_signal; zeros(N-length(upsampled_signal), 1)];
    end
    
    % 脉冲成形滤波
    rolloff = 0.25;
    span = 6;
    rrcfilter = rcosdesign(rolloff, span, samples_per_symbol);
    baseband_signal = conv(upsampled_signal, rrcfilter, 'same');
    
    % 信道模型
    if strcmp(channel_type, 'LoS')
        path_loss = 96; % dB
        K_factor = 10; % Rician K因子
        K_linear = 10^(K_factor/10);
        rayleigh_component = (randn(N,1) + 1j*randn(N,1)) / sqrt(2);
        los_component = sqrt(K_linear / (K_linear + 1));
        nlos_component = sqrt(1 / (K_linear + 1)) * rayleigh_component;
        fading_coeff = los_component + nlos_component;
    else
        path_loss = 110; % dB
        fading_coeff = (randn(N,1) + 1j*randn(N,1)) / sqrt(2);
    end
    
    channel_gain = 10^(-path_loss/20);
    received_signal_baseband = baseband_signal * channel_gain .* fading_coeff;
    
    % 载波调制
    t = (0:N-1)' / fs;
    carrier = exp(1j*2*pi*fc*t);
    transmitted_signal = real(received_signal_baseband .* carrier);
    
    % 添加噪声
    noise = sqrt(noise_power/2) * (randn(N,1) + 1j*randn(N,1));
    received_signal = transmitted_signal + real(noise);
    
    % 载波解调
    demodulated_signal = received_signal .* cos(2*pi*fc*t);
    [b, a] = butter(6, bandwidth/(fs/2));
    received_baseband = filter(b, a, demodulated_signal);
    
    % 参数输出
    params.fs = fs;
    params.fc = fc;
    params.path_loss = path_loss;
    params.noise_power = noise_power;
    params.signal_power = P_tx_linear;
end

function analysis = analyze_signal_performance(received_signal, received_baseband, fs, params)
    % 分析信号性能
    
    N = length(received_signal);
    
    % 计算SNR
    signal_power = var(real(received_baseband));
    noise_power = params.noise_power;
    snr_db = 10*log10(signal_power / noise_power);
    
    % 计算EVM (Error Vector Magnitude)
    % 简化的EVM计算
    evm_percent = sqrt(noise_power / signal_power) * 100;
    
    % 频谱效率
    bandwidth = 20e6;
    data_rate = 1e6;
    spectral_efficiency = data_rate / bandwidth;
    
    % 峰均功率比 (PAPR)
    papr_db = 10*log10(max(abs(received_signal).^2) / mean(abs(received_signal).^2));
    
    % 频谱泄漏分析
    [psd, f_psd] = pwelch(received_signal, hamming(1024), 512, 1024, fs);
    total_power = trapz(f_psd, psd);
    
    % 带内功率 (假设有用信号带宽为20MHz)
    useful_band_idx = abs(f_psd) <= 10e6;
    inband_power = trapz(f_psd(useful_band_idx), psd(useful_band_idx));
    spectral_leakage = (total_power - inband_power) / total_power * 100;
    
    % 存储分析结果
    analysis.snr_db = snr_db;
    analysis.evm_percent = evm_percent;
    analysis.spectral_efficiency = spectral_efficiency;
    analysis.papr_db = papr_db;
    analysis.spectral_leakage_percent = spectral_leakage;
    analysis.total_power = total_power;
    analysis.inband_power = inband_power;
    analysis.psd = psd;
    analysis.f_psd = f_psd;
end

function plot_performance_comparison(results, oversample_factors, channel_types)
    % 绘制性能比较图
    
    figure('Position', [100, 100, 1400, 800]);
    
    % 提取数据
    snr_data = zeros(length(channel_types), length(oversample_factors));
    evm_data = zeros(length(channel_types), length(oversample_factors));
    papr_data = zeros(length(channel_types), length(oversample_factors));
    leakage_data = zeros(length(channel_types), length(oversample_factors));
    
    for ch_idx = 1:length(channel_types)
        for os_idx = 1:length(oversample_factors)
            key = sprintf('%s_OS%d', channel_types{ch_idx}, oversample_factors(os_idx));
            snr_data(ch_idx, os_idx) = results.(key).snr_db;
            evm_data(ch_idx, os_idx) = results.(key).evm_percent;
            papr_data(ch_idx, os_idx) = results.(key).papr_db;
            leakage_data(ch_idx, os_idx) = results.(key).spectral_leakage_percent;
        end
    end
    
    % SNR比较
    subplot(2,2,1);
    bar_width = 0.35;
    x = 1:length(oversample_factors);
    bar(x - bar_width/2, snr_data(1,:), bar_width, 'DisplayName', 'LoS');
    hold on;
    bar(x + bar_width/2, snr_data(2,:), bar_width, 'DisplayName', 'NLoS');
    xlabel('过采样因子');
    ylabel('SNR (dB)');
    title('信噪比比较');
    legend();
    grid on;
    set(gca, 'XTickLabel', cellstr(num2str(oversample_factors')));
    
    % EVM比较
    subplot(2,2,2);
    bar(x - bar_width/2, evm_data(1,:), bar_width, 'DisplayName', 'LoS');
    hold on;
    bar(x + bar_width/2, evm_data(2,:), bar_width, 'DisplayName', 'NLoS');
    xlabel('过采样因子');
    ylabel('EVM (%)');
    title('误差向量幅度比较');
    legend();
    grid on;
    set(gca, 'XTickLabel', cellstr(num2str(oversample_factors')));
    
    % PAPR比较
    subplot(2,2,3);
    bar(x - bar_width/2, papr_data(1,:), bar_width, 'DisplayName', 'LoS');
    hold on;
    bar(x + bar_width/2, papr_data(2,:), bar_width, 'DisplayName', 'NLoS');
    xlabel('过采样因子');
    ylabel('PAPR (dB)');
    title('峰均功率比比较');
    legend();
    grid on;
    set(gca, 'XTickLabel', cellstr(num2str(oversample_factors')));
    
    % 频谱泄漏比较
    subplot(2,2,4);
    bar(x - bar_width/2, leakage_data(1,:), bar_width, 'DisplayName', 'LoS');
    hold on;
    bar(x + bar_width/2, leakage_data(2,:), bar_width, 'DisplayName', 'NLoS');
    xlabel('过采样因子');
    ylabel('频谱泄漏 (%)');
    title('频谱泄漏比较');
    legend();
    grid on;
    set(gca, 'XTickLabel', cellstr(num2str(oversample_factors')));
    
    sgtitle('UAV通信系统性能参数比较');
end

function generate_performance_report(results, oversample_factors, channel_types)
    % 生成性能报告
    
    fprintf('\n=== 性能分析报告 ===\n');
    fprintf('%-10s %-8s %-8s %-8s %-8s %-12s\n', '信道类型', '过采样', 'SNR(dB)', 'EVM(%)', 'PAPR(dB)', '频谱泄漏(%)');
    fprintf('%s\n', repmat('-', 1, 70));
    
    for ch_idx = 1:length(channel_types)
        for os_idx = 1:length(oversample_factors)
            key = sprintf('%s_OS%d', channel_types{ch_idx}, oversample_factors(os_idx));
            r = results.(key);
            fprintf('%-10s %-8d %-8.2f %-8.2f %-8.2f %-12.2f\n', ...
                channel_types{ch_idx}, oversample_factors(os_idx), ...
                r.snr_db, r.evm_percent, r.papr_db, r.spectral_leakage_percent);
        end
    end
    
    % 保存结果到文件
    save('uav_parameter_analysis_results.mat', 'results', 'oversample_factors', 'channel_types');
    fprintf('\n分析结果已保存到 uav_parameter_analysis_results.mat\n');
end