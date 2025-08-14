%% UAV分布式频谱感知系统仿真
% 基于A2G信道模型的多UAV频谱感知仿真
% 包含PU Markov状态转移、LoS/NLoS信道、多UAV部署等

clear all; close all; clc;

%% 系统参数设置
fprintf('=== UAV分布式频谱感知系统仿真 ===\n');

% 基本系统参数
fs = 30.72e6;           % 采样频率 30.72 MHz (LTE/5G标准)
T_sense = 6.5e-6;       % 感知周期 6.5 μs
N_samples = 200;        % 每周期采样点数
N_uav = 10;             % UAV数量
N_time_slots = 100;     % 仿真时隙数量

% PU参数
P_tx_dbm = 20;          % PU发射功率 20 dBm
P_tx = 10^(P_tx_dbm/10) * 1e-3;  % 转换为瓦特

% 噪声参数
noise_psd_dbm = -174;   % 噪声功率谱密度 -174 dBm/Hz
B_rx = 20e6;            % 接收系统带宽 20 MHz
F_db = 1;               % 噪声因子 1 dB
F_linear = 10^(F_db/10);
noise_power = 10^(noise_psd_dbm/10) * 1e-3 * B_rx * F_linear;

% A2G信道参数
% LoS参数
alpha_los = 2;          % LoS路径损耗指数
beta_los = 0;           % LoS路径损耗因子 (dB)
% NLoS参数  
alpha_nlos = 4;         % NLoS路径损耗指数
beta_nlos = -20;        % NLoS路径损耗因子 (dB)
% 环境常数 (Al-Hourani模型)
A_env = 9.6;
B_env = 0.16;

% Markov状态转移概率矩阵
% PU平均活动时间 = 5时隙，平均空闲时间 = 10时隙
P_off_off = 9/10;       % 空闲保持空闲概率
P_off_on = 1/10;        % 空闲转活动概率
P_on_off = 1/5;         % 活动转空闲概率
P_on_on = 4/5;          % 活动保持活动概率

P_markov = [P_off_off, P_off_on; P_on_off, P_on_on];

fprintf('系统参数:\n');
fprintf('  采样频率: %.2f MHz\n', fs/1e6);
fprintf('  感知周期: %.1f μs\n', T_sense*1e6);
fprintf('  每周期采样数: %d\n', N_samples);
fprintf('  UAV数量: %d\n', N_uav);
fprintf('  PU发射功率: %.1f dBm\n', P_tx_dbm);
fprintf('  噪声功率: %.2e W\n', noise_power);

%% UAV位置部署
fprintf('\n部署UAV位置...\n');

% 随机部署UAV位置
rng(42); % 设置随机种子以便复现结果

% UAV水平距离 (0, 1000] 米
uav_horizontal_dist = 50 + (1000-50) * rand(N_uav, 1);

% UAV高度 [300, 320] 米
uav_height = 300 + 20 * rand(N_uav, 1);

% PU位置设为原点
pu_position = [0, 0, 0];  % [x, y, z]

% 计算UAV到PU的3D距离
uav_positions = zeros(N_uav, 3);
distances_3d = zeros(N_uav, 1);
elevation_angles = zeros(N_uav, 1);

for i = 1:N_uav
    % 随机角度部署UAV
    angle = 2*pi*rand();
    uav_positions(i, :) = [uav_horizontal_dist(i)*cos(angle), ...
                          uav_horizontal_dist(i)*sin(angle), ...
                          uav_height(i)];
    
    % 计算3D距离和仰角
    distances_3d(i) = sqrt(sum((uav_positions(i, :) - pu_position).^2));
    elevation_angles(i) = atan(uav_height(i) / uav_horizontal_dist(i));
end

fprintf('UAV部署完成:\n');
for i = 1:N_uav
    fprintf('  UAV%d: 距离=%.1fm, 高度=%.1fm, 仰角=%.1f°\n', ...
        i, uav_horizontal_dist(i), uav_height(i), rad2deg(elevation_angles(i)));
end

%% 计算LoS概率和信道增益
fprintf('\n计算A2G信道参数...\n');

% 计算LoS概率 (Al-Hourani模型)
P_los = zeros(N_uav, 1);
for i = 1:N_uav
    theta_deg = rad2deg(elevation_angles(i));
    P_los(i) = 1 / (1 + A_env * exp(-B_env * (theta_deg - A_env)));
end

% 计算大尺度衰落 (路径损耗)
path_loss_los = zeros(N_uav, 1);
path_loss_nlos = zeros(N_uav, 1);
channel_gains = zeros(N_uav, N_time_slots);
channel_types = cell(N_uav, N_time_slots);

for i = 1:N_uav
    % LoS和NLoS路径损耗
    path_loss_los(i) = -20*log10(4*pi*distances_3d(i)/3e8) - alpha_los*10*log10(distances_3d(i)) - beta_los;
    path_loss_nlos(i) = -20*log10(4*pi*distances_3d(i)/3e8) - alpha_nlos*10*log10(distances_3d(i)) - beta_nlos;
    
    % 每个时隙随机选择LoS或NLoS
    for t = 1:N_time_slots
        if rand() < P_los(i)
            % LoS信道
            channel_types{i, t} = 'LoS';
            large_scale = 10^(path_loss_los(i)/20);
            % Rician小尺度衰落 (K=10dB)
            K_factor = 10; % dB
            K_linear = 10^(K_factor/10);
            h_los = sqrt(K_linear/(K_linear+1));
            h_nlos = sqrt(1/(K_linear+1)) * (randn() + 1j*randn())/sqrt(2);
            small_scale = h_los + h_nlos;
        else
            % NLoS信道
            channel_types{i, t} = 'NLoS';
            large_scale = 10^(path_loss_nlos(i)/20);
            % Rayleigh小尺度衰落
            small_scale = (randn() + 1j*randn())/sqrt(2);
        end
        
        channel_gains(i, t) = large_scale * small_scale;
    end
end

%% 生成PU Markov状态序列
fprintf('生成PU状态序列...\n');

% 初始状态随机选择
pu_state = zeros(1, N_time_slots);
pu_state(1) = randi([0, 1]);

% 根据Markov转移矩阵生成状态序列
for t = 2:N_time_slots
    if pu_state(t-1) == 0  % 当前OFF
        if rand() < P_off_on
            pu_state(t) = 1;  % 转为ON
        else
            pu_state(t) = 0;  % 保持OFF
        end
    else  % 当前ON
        if rand() < P_on_off
            pu_state(t) = 0;  % 转为OFF
        else
            pu_state(t) = 1;  % 保持ON
        end
    end
end

fprintf('PU状态统计: ON=%.1f%%, OFF=%.1f%%\n', ...
    mean(pu_state)*100, (1-mean(pu_state))*100);

%% 生成接收信号和计算SNR
fprintf('生成接收信号...\n');

% 时间向量
t_sample = (0:N_samples-1) / fs;

% 存储结果
received_signals = zeros(N_uav, N_samples, N_time_slots);
snr_values = zeros(N_uav, N_time_slots);
detection_results = zeros(N_uav, N_time_slots);

% 生成PU发射信号 (QPSK调制)
symbol_rate = 1e6;  % 1 Msps
samples_per_symbol = round(fs / symbol_rate);
num_symbols = ceil(N_samples / samples_per_symbol);

for t = 1:N_time_slots
    % 生成PU信号
    if pu_state(t) == 1  % PU活动
        % 生成QPSK符号
        data_bits = randi([0 1], num_symbols*2, 1);
        I_data = 2*data_bits(1:2:end) - 1;
        Q_data = 2*data_bits(2:2:end) - 1;
        symbols = (I_data + 1j*Q_data) / sqrt(2);
        symbols = symbols * sqrt(P_tx);
        
        % 上采样
        pu_signal = upsample(symbols, samples_per_symbol);
        pu_signal = pu_signal(1:N_samples);
        if length(pu_signal) < N_samples
            pu_signal = [pu_signal; zeros(N_samples-length(pu_signal), 1)];
        end
        
        % 简单脉冲成形
        if samples_per_symbol >= 4
            h_filter = ones(min(4, samples_per_symbol), 1) / min(4, samples_per_symbol);
            pu_signal = conv(pu_signal, h_filter, 'same');
        end
    else  % PU空闲
        pu_signal = zeros(N_samples, 1);
    end
    
    % 计算每个UAV的接收信号
    for i = 1:N_uav
        % 信号传播延迟
        c = 3e8;  % 光速
        delay_time = distances_3d(i) / c;
        delay_samples = round(delay_time * fs);
        
        % 应用信道增益
        received_signal_clean = channel_gains(i, t) * pu_signal;
        
        % 应用传播延迟
        if delay_samples > 0 && delay_samples < N_samples
            temp_signal = [zeros(delay_samples, 1); received_signal_clean];
            received_signal_clean = temp_signal(1:N_samples);
        end
        
        % 添加噪声
        noise = sqrt(noise_power/2) * (randn(N_samples, 1) + 1j*randn(N_samples, 1));
        received_signals(i, :, t) = received_signal_clean + noise;
        
        % 计算SNR
        if pu_state(t) == 1
            signal_power = abs(channel_gains(i, t))^2 * P_tx;
            snr_values(i, t) = 10*log10(signal_power / noise_power);
        else
            snr_values(i, t) = -Inf;  % 无信号时SNR为负无穷
        end
        
        % 简单能量检测
        energy = mean(abs(received_signals(i, :, t)).^2);
        threshold = 2 * noise_power;  % 简单阈值
        detection_results(i, t) = (energy > threshold);
    end
end

%% 性能分析
fprintf('\n=== 性能分析结果 ===\n');

% 计算平均SNR (仅PU活动时)
active_slots = find(pu_state == 1);
if ~isempty(active_slots)
    avg_snr_per_uav = zeros(N_uav, 1);
    for i = 1:N_uav
        valid_snr = snr_values(i, active_slots);
        valid_snr = valid_snr(isfinite(valid_snr));
        if ~isempty(valid_snr)
            avg_snr_per_uav(i) = mean(valid_snr);
        else
            avg_snr_per_uav(i) = -Inf;
        end
    end
    
    fprintf('各UAV平均SNR (PU活动时):\n');
    for i = 1:N_uav
        if isfinite(avg_snr_per_uav(i))
            fprintf('  UAV%d: %.2f dB\n', i, avg_snr_per_uav(i));
        else
            fprintf('  UAV%d: 无有效信号\n', i);
        end
    end
end

% 计算检测性能
total_detections = sum(detection_results, 2);
detection_rate = total_detections / N_time_slots * 100;

fprintf('\n各UAV检测率:\n');
for i = 1:N_uav
    fprintf('  UAV%d: %.1f%%\n', i, detection_rate(i));
end

% 计算LoS/NLoS统计
los_count = zeros(N_uav, 1);
for i = 1:N_uav
    for t = 1:N_time_slots
        if strcmp(channel_types{i, t}, 'LoS')
            los_count(i) = los_count(i) + 1;
        end
    end
end
los_percentage = los_count / N_time_slots * 100;

fprintf('\n各UAV LoS概率:\n');
for i = 1:N_uav
    fprintf('  UAV%d: 理论=%.1f%%, 实际=%.1f%%\n', i, P_los(i)*100, los_percentage(i));
end

%% 绘制结果
fprintf('\n绘制分析图表...\n');

% 图1: UAV部署图
figure('Position', [100, 100, 1600, 1200]);

subplot(3,3,1);
scatter3(uav_positions(:,1), uav_positions(:,2), uav_positions(:,3), 100, 'r', 'filled');
hold on;
plot3(0, 0, 0, 'ko', 'MarkerSize', 15, 'MarkerFaceColor', 'k');
for i = 1:N_uav
    text(uav_positions(i,1), uav_positions(i,2), uav_positions(i,3)+10, ...
        sprintf('UAV%d', i), 'FontSize', 8);
end
xlabel('X (m)'); ylabel('Y (m)'); zlabel('高度 (m)');
title('UAV分布式部署');
grid on;

% 图2: PU状态序列
subplot(3,3,2);
plot(1:N_time_slots, pu_state, 'b-', 'LineWidth', 2);
xlabel('时隙');
ylabel('PU状态');
title('PU Markov状态序列');
ylim([-0.1, 1.1]);
grid on;

% 图3: SNR分布
subplot(3,3,3);
if ~isempty(active_slots)
    valid_uavs = find(isfinite(avg_snr_per_uav));
    if ~isempty(valid_uavs)
        bar(valid_uavs, avg_snr_per_uav(valid_uavs));
        xlabel('UAV编号');
        ylabel('平均SNR (dB)');
        title('各UAV平均SNR');
        grid on;
    end
end

% 图4: 距离vs SNR
subplot(3,3,4);
if ~isempty(active_slots)
    valid_uavs = find(isfinite(avg_snr_per_uav));
    if ~isempty(valid_uavs)
        scatter(distances_3d(valid_uavs), avg_snr_per_uav(valid_uavs), 100, 'filled');
        xlabel('距离 (m)');
        ylabel('平均SNR (dB)');
        title('距离 vs SNR关系');
        grid on;
    end
end

% 图5: LoS概率比较
subplot(3,3,5);
bar([P_los*100, los_percentage], 'grouped');
xlabel('UAV编号');
ylabel('LoS概率 (%)');
title('LoS概率: 理论 vs 实际');
legend('理论', '实际');
grid on;

% 图6: 检测率
subplot(3,3,6);
bar(1:N_uav, detection_rate);
xlabel('UAV编号');
ylabel('检测率 (%)');
title('各UAV检测率');
grid on;

% 图7: 时域信号示例 (UAV1, 前几个时隙)
subplot(3,3,7);
plot_slots = min(5, N_time_slots);
for t = 1:plot_slots
    plot(t_sample*1e6, real(squeeze(received_signals(1, :, t))), ...
        'DisplayName', sprintf('时隙%d', t));
    hold on;
end
xlabel('时间 (μs)');
ylabel('幅度');
title('UAV1接收信号时域 (前5时隙)');
legend('Location', 'best');
grid on;

% 图8: 频域信号示例
subplot(3,3,8);
if any(pu_state(1:min(5,N_time_slots)))
    active_slot = find(pu_state(1:min(5,N_time_slots)), 1);
    if ~isempty(active_slot)
        signal_fft = fftshift(fft(squeeze(received_signals(1, :, active_slot))));
        f_axis = (-N_samples/2:N_samples/2-1) * (fs/N_samples) / 1e6;
        plot(f_axis, 20*log10(abs(signal_fft) + eps));
        xlabel('频率 (MHz)');
        ylabel('幅度 (dB)');
        title(sprintf('UAV1接收信号频谱 (时隙%d)', active_slot));
        grid on;
    end
end

% 图9: 信道增益时变
subplot(3,3,9);
plot(1:N_time_slots, 20*log10(abs(channel_gains(1:min(3,N_uav), :))));
xlabel('时隙');
ylabel('信道增益 (dB)');
title('信道增益时变特性');
legend(arrayfun(@(x) sprintf('UAV%d', x), 1:min(3,N_uav), 'UniformOutput', false));
grid on;

sgtitle('UAV分布式频谱感知系统仿真结果');

%% 保存结果
save('uav_distributed_sensing_results.mat', ...
    'uav_positions', 'distances_3d', 'elevation_angles', 'P_los', ...
    'channel_gains', 'channel_types', 'pu_state', 'received_signals', ...
    'snr_values', 'detection_results', 'avg_snr_per_uav', 'detection_rate');

fprintf('\n=== 仿真完成 ===\n');
fprintf('结果已保存到 uav_distributed_sensing_results.mat\n');
fprintf('总仿真时隙: %d\n', N_time_slots);
fprintf('PU活动率: %.1f%%\n', mean(pu_state)*100);
fprintf('平均检测率: %.1f%%\n', mean(detection_rate));