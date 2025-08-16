%% UAV通信系统主仿真程序
% 严格按照图片中的场景和参数进行仿真
% 信号处理链路：成型滤波 → 上变频 → 信道 → 下变频 → 匹配滤波

clear all; close all; clc;

%% 加载配置参数
config = uav_config();

% 设置随机种子以获得可重复结果
rng(config.simulation.seed);

%% 计算派生参数
fc = config.communication.fc;
fs = fc * config.simulation.oversample_factor;  % 采样频率
symbol_rate = config.communication.symbol_rate;
sps = round(fs / symbol_rate);  % 每符号采样点数

% 几何计算
distance_3d = sqrt(config.geometry.distance_2d^2 + ...
                  (config.geometry.uav_height - config.geometry.base_station_height)^2);
elevation_angle = atan((config.geometry.uav_height - config.geometry.base_station_height) / ...
                      config.geometry.distance_2d);

% 多普勒频移计算
if config.simulation.enable_doppler
    doppler_freq = fc * config.geometry.uav_speed * cos(elevation_angle) / 3e8;
else
    doppler_freq = 0;
end

fprintf('=== 仿真开始 ===\n');
fprintf('3D距离: %.2f m, 仰角: %.2f°\n', distance_3d, elevation_angle*180/pi);
fprintf('多普勒频移: %.2f Hz\n', doppler_freq);

%% 1. 生成基带数据信号
num_bits = config.simulation.num_symbols * 2; % QPSK需要2 bits/symbol
data_bits = randi([0,1], num_bits, 1);

% QPSK调制
qpsk_symbols = zeros(config.simulation.num_symbols, 1);
for i = 1:config.simulation.num_symbols
    bit_pair = data_bits(2*i-1:2*i);
    if isequal(bit_pair, [0;0])
        qpsk_symbols(i) = (1+1j)/sqrt(2);
    elseif isequal(bit_pair, [0;1])
        qpsk_symbols(i) = (-1+1j)/sqrt(2);
    elseif isequal(bit_pair, [1;0])
        qpsk_symbols(i) = (1-1j)/sqrt(2);
    else  % [1;1]
        qpsk_symbols(i) = (-1-1j)/sqrt(2);
    end
end

%% 2. 成型滤波（发射端脉冲成型）
% 设计根升余弦滤波器
rrc_filter = rcosdesign(config.communication.rolloff, ...
                       config.communication.filter_span, sps, 'sqrt');

% 上采样并进行脉冲成型
upsampled_signal = upsample(qpsk_symbols, sps);
tx_baseband = conv(upsampled_signal, rrc_filter, 'same');

fprintf('成型滤波完成\n');

%% 3. 上变频到射频
t = (0:length(tx_baseband)-1)' / fs;
carrier = exp(1j*2*pi*fc*t);

% I/Q上变频
tx_rf = real(tx_baseband .* carrier);

fprintf('上变频完成，载波频率: %.2f GHz\n', fc/1e9);

%% 4. 信道建模
% LoS路径损耗（基于图片中的公式）
path_loss_dB = config.channel.los_A * log10(distance_3d) + ...
               config.channel.los_B + ...
               20*log10(fc/1e9);

% 天线增益
total_gain_dB = config.antenna.gain_tx + config.antenna.gain_rx;

% 总的信道增益
channel_gain_dB = -path_loss_dB + total_gain_dB;
channel_gain = 10^(channel_gain_dB/20);

% 应用信道衰落
rx_signal_no_noise = tx_rf * channel_gain;

% 多普勒效应
if config.simulation.enable_doppler
    doppler_phase = 2*pi*doppler_freq*t;
    doppler_effect = cos(doppler_phase);
    rx_signal_no_noise = rx_signal_no_noise .* doppler_effect;
end

% 计算噪声功率
signal_power = mean(abs(rx_signal_no_noise).^2);
noise_power = signal_power / (10^(config.channel.SNR_dB/10));

% 添加高斯白噪声
noise = sqrt(noise_power/2) * randn(size(rx_signal_no_noise));
rx_rf = rx_signal_no_noise + noise;

fprintf('信道建模完成，路径损耗: %.2f dB\n', path_loss_dB);

%% 5. 下变频
local_oscillator = exp(-1j*2*pi*fc*t);
rx_baseband_complex = rx_rf .* local_oscillator;

% 低通滤波去除镜像频率
cutoff_freq = config.frequency.bandwidth / 2;
[b, a] = butter(6, cutoff_freq/(fs/2));
rx_baseband_filtered = filter(b, a, rx_baseband_complex);

fprintf('下变频完成\n');

%% 6. 匹配滤波
matched_filter = conj(fliplr(rrc_filter'));
rx_matched = conv(rx_baseband_filtered, matched_filter, 'same');

% 下采样恢复符号速率
rx_symbols = downsample(rx_matched, sps);

% 长度对齐
min_len = min(length(rx_symbols), length(qpsk_symbols));
rx_symbols = rx_symbols(1:min_len);
original_symbols = qpsk_symbols(1:min_len);

fprintf('匹配滤波完成\n');

%% 7. 性能评估
% EVM计算
evm_rms = sqrt(mean(abs(rx_symbols - original_symbols).^2)) / ...
          sqrt(mean(abs(original_symbols).^2));
evm_percent = evm_rms * 100;

% 符号错误率估计
symbol_errors = sum(abs(rx_symbols - original_symbols) > 0.5);
ser = symbol_errors / min_len;

fprintf('=== 性能指标 ===\n');
fprintf('EVM: %.2f%%\n', evm_percent);
fprintf('符号错误率: %.4f\n', ser);

%% 8. 生成图形结果
% 时域分析图
figure('Name', 'UAV通信系统时域分析', 'Position', [100, 100, 1400, 900]);

subplot(3,3,1);
plot(real(tx_baseband(1:min(1000, length(tx_baseband)))));
title('发射基带信号（成型后）');
xlabel('采样点'); ylabel('幅度');
grid on;

subplot(3,3,2);
plot(tx_rf(1:min(1000, length(tx_rf))));
title('发射射频信号');
xlabel('采样点'); ylabel('幅度');
grid on;

subplot(3,3,3);
plot(rx_rf(1:min(1000, length(rx_rf))));
title('接收射频信号（含噪声）');
xlabel('采样点'); ylabel('幅度');
grid on;

subplot(3,3,4);
plot(real(rx_baseband_filtered(1:min(1000, length(rx_baseband_filtered)))));
title('下变频后基带信号');
xlabel('采样点'); ylabel('幅度');
grid on;

subplot(3,3,5);
plot(real(rx_matched(1:min(1000, length(rx_matched)))));
title('匹配滤波后信号');
xlabel('采样点'); ylabel('幅度');
grid on;

% 频域分析
N = min(8192, length(tx_rf));  % 限制FFT长度以提高效率
f = (-N/2:N/2-1) * fs/N;

tx_spectrum = fftshift(fft(tx_rf(1:N)));
rx_spectrum = fftshift(fft(rx_rf(1:N)));

subplot(3,3,6);
plot(f/1e9, 20*log10(abs(tx_spectrum) + eps));
title('发射信号频谱');
xlabel('频率 (GHz)'); ylabel('幅度 (dB)');
grid on;

subplot(3,3,7);
plot(f/1e9, 20*log10(abs(rx_spectrum) + eps));
title('接收信号频谱');
xlabel('频率 (GHz)'); ylabel('幅度 (dB)');
grid on;

% 功率谱密度
window_len = min(1024, floor(length(rx_rf)/4));
[psd_rx, f_psd] = pwelch(rx_rf, window_len, [], [], fs);

subplot(3,3,8);
semilogy(f_psd/1e9, psd_rx);
title('接收信号功率谱密度');
xlabel('频率 (GHz)'); ylabel('PSD (W/Hz)');
grid on;

% 星座图
subplot(3,3,9);
scatter(real(rx_symbols), imag(rx_symbols), 20, 'r', 'filled', 'alpha', 0.6);
hold on;
scatter(real(original_symbols), imag(original_symbols), 20, 'b', 'x', 'LineWidth', 2);
title('星座图对比');
xlabel('I'); ylabel('Q');
legend('接收', '发射', 'Location', 'best');
grid on; axis equal;

% 性能分析图
figure('Name', 'UAV通信系统性能分析', 'Position', [1500, 100, 800, 600]);

subplot(2,2,1);
plot(1:min_len, abs(rx_symbols - original_symbols));
title(['误差向量幅度 (EVM: ' sprintf('%.2f%%', evm_percent) ')']);
xlabel('符号索引'); ylabel('EVM');
grid on;

subplot(2,2,2);
histogram(abs(rx_symbols - original_symbols), 50);
title('误差分布');
xlabel('误差幅度'); ylabel('频次');
grid on;

subplot(2,2,3);
plot(real(rx_symbols(1:100)), 'r-', 'LineWidth', 1.5);
hold on;
plot(real(original_symbols(1:100)), 'b--', 'LineWidth', 1.5);
title('信号恢复对比（实部）');
xlabel('符号索引'); ylabel('幅度');
legend('接收', '发射', 'Location', 'best');
grid on;

subplot(2,2,4);
stem(0, abs(channel_gain), 'filled', 'LineWidth', 2);
title(['信道增益: ' sprintf('%.2f dB', channel_gain_dB)]);
xlabel('时延'); ylabel('幅度');
grid on;

%% 9. 保存结果
results = struct();
results.config = config;
results.performance.evm_percent = evm_percent;
results.performance.ser = ser;
results.performance.path_loss_dB = path_loss_dB;
results.performance.channel_gain_dB = channel_gain_dB;
results.performance.doppler_freq = doppler_freq;
results.signals.tx_baseband = tx_baseband;
results.signals.tx_rf = tx_rf;
results.signals.rx_rf = rx_rf;
results.signals.rx_symbols = rx_symbols;
results.signals.original_symbols = original_symbols;

save('uav_simulation_results.mat', 'results');

fprintf('=== 仿真完成 ===\n');
fprintf('结果已保存到 uav_simulation_results.mat\n');
fprintf('生成了时域、频域和功率谱密度图\n');