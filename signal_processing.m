function [tx_signal, rx_signals, tx_baseband, rx_baseband] = signal_processing(pu_states, channel_gains, delays, params)
%% 信号处理流程
% 输入:
%   pu_states - PU状态序列
%   channel_gains - 信道增益
%   delays - 传播延迟
%   params - 仿真参数结构体
% 输出:
%   tx_signal - 发射信号 (射频)
%   rx_signals - 接收信号 (射频, 各UAV)
%   tx_baseband - 发射基带信号
%   rx_baseband - 接收基带信号 (各UAV)

%% 1. 生成原始数据序列
fprintf('正在生成PU信号...\n');

% 根据PU状态生成数据
num_active_symbols = sum(pu_states);
if num_active_symbols == 0
    error('PU在整个仿真期间都处于空闲状态');
end

% 生成QPSK调制符号
data_bits = randi([0 1], num_active_symbols * 2, 1); % 每个QPSK符号2个bit
qpsk_symbols = qpsk_modulate(data_bits);

fprintf('  生成了 %d 个QPSK符号\n', length(qpsk_symbols));

%% 2. 上采样和成型滤波
fprintf('正在进行成型滤波...\n');

% 上采样
upsampled_symbols = upsample(qpsk_symbols, params.oversample_factor);

% 设计升余弦滤波器
filter_length = 8 * params.oversample_factor; % 滤波器长度
rrc_filter = rcosdesign(params.rolloff_factor, filter_length, params.oversample_factor, 'sqrt');

% 成型滤波
tx_baseband_temp = conv(upsampled_symbols, rrc_filter, 'same');

% 调整长度以匹配时间槽
samples_per_slot = params.oversample_factor;
total_samples = length(pu_states) * samples_per_slot;

% 生成完整的基带信号（包含空闲时段）
tx_baseband = zeros(total_samples, 1);
active_sample_idx = 1;

for t = 1:length(pu_states)
    start_idx = (t-1) * samples_per_slot + 1;
    end_idx = t * samples_per_slot;
    
    if pu_states(t) == 1 % PU活动
        if active_sample_idx <= length(tx_baseband_temp)
            tx_baseband(start_idx:end_idx) = tx_baseband_temp(active_sample_idx:min(active_sample_idx+samples_per_slot-1, length(tx_baseband_temp)));
            active_sample_idx = active_sample_idx + samples_per_slot;
        end
    end
    % 空闲时段保持为0
end

fprintf('  基带信号长度: %d 样本\n', length(tx_baseband));

%% 3. 上变频到射频
fprintf('正在进行上变频...\n');

% 时间向量
dt = 1 / params.fs;
t = (0:length(tx_baseband)-1)' * dt;

% 上变频 (只考虑实部，简化处理)
tx_signal = real(tx_baseband .* exp(1j * 2 * pi * params.fc * t));

fprintf('  射频信号生成完成\n');

%% 4. 信道传输
fprintf('正在模拟信道传输...\n');

num_uav = length(channel_gains);
rx_signals = zeros(length(tx_signal), num_uav);

for uav_idx = 1:num_uav
    % 计算延迟样本数
    delay_samples = round(delays(uav_idx) * params.fs);
    
    % 应用信道增益和延迟
    delayed_signal = [zeros(delay_samples, 1); tx_signal];
    if length(delayed_signal) > length(tx_signal)
        delayed_signal = delayed_signal(1:length(tx_signal));
    end
    
    % 信道增益 (只使用幅度，相位在基带处理)
    channel_gain_magnitude = abs(channel_gains(uav_idx));
    rx_signals(:, uav_idx) = channel_gain_magnitude * delayed_signal;
end

fprintf('  %d 个UAV的接收信号生成完成\n', num_uav);

%% 5. 下变频到基带
fprintf('正在进行下变频...\n');

rx_baseband = zeros(length(tx_signal), num_uav);

for uav_idx = 1:num_uav
    % 下变频
    baseband_complex = rx_signals(:, uav_idx) .* exp(-1j * 2 * pi * params.fc * t);
    
    % 低通滤波 (简化：使用相同的RRC滤波器)
    rx_baseband(:, uav_idx) = conv(baseband_complex, rrc_filter, 'same');
    
    % 应用信道的复数增益
    rx_baseband(:, uav_idx) = rx_baseband(:, uav_idx) * (channel_gains(uav_idx) / abs(channel_gains(uav_idx)));
end

fprintf('  基带信号恢复完成\n');

%% 6. 添加噪声 (在基带)
fprintf('正在添加噪声...\n');

% 计算噪声功率
noise_power_dbm = params.noise_psd_dbm_hz + 10*log10(params.bandwidth) + params.noise_figure_db;
noise_power_watts = 10^((noise_power_dbm - 30)/10);
noise_std = sqrt(noise_power_watts / 2); % 复数噪声的标准差

for uav_idx = 1:num_uav
    % 添加复数高斯白噪声
    noise_real = noise_std * randn(size(rx_baseband, 1), 1);
    noise_imag = noise_std * randn(size(rx_baseband, 1), 1);
    noise = noise_real + 1j * noise_imag;
    
    rx_baseband(:, uav_idx) = rx_baseband(:, uav_idx) + noise;
end

fprintf('  噪声添加完成\n');
fprintf('  噪声功率: %.1f dBm\n', noise_power_dbm);

fprintf('信号处理流程完成！\n\n');

end

%% 辅助函数：QPSK调制
function symbols = qpsk_modulate(bits)
    % 将比特转换为QPSK符号
    % 00 -> (1+1j)/sqrt(2)
    % 01 -> (-1+1j)/sqrt(2)  
    % 10 -> (1-1j)/sqrt(2)
    % 11 -> (-1-1j)/sqrt(2)
    
    if mod(length(bits), 2) ~= 0
        error('比特数必须是偶数');
    end
    
    % 将比特重新排列为2列
    bit_pairs = reshape(bits, 2, [])';
    
    % 映射到QPSK符号
    symbols = zeros(size(bit_pairs, 1), 1);
    
    for i = 1:size(bit_pairs, 1)
        b1 = bit_pairs(i, 1);
        b2 = bit_pairs(i, 2);
        
        if b1 == 0 && b2 == 0
            symbols(i) = (1 + 1j) / sqrt(2);
        elseif b1 == 0 && b2 == 1
            symbols(i) = (-1 + 1j) / sqrt(2);
        elseif b1 == 1 && b2 == 0
            symbols(i) = (1 - 1j) / sqrt(2);
        else % b1 == 1 && b2 == 1
            symbols(i) = (-1 - 1j) / sqrt(2);
        end
    end
end