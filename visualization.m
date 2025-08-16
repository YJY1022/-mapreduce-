function visualization(tx_signal, rx_signals, tx_baseband, rx_baseband, pu_states, params)
%% 仿真结果可视化
% 输入:
%   tx_signal - 发射射频信号
%   rx_signals - 接收射频信号 (各UAV)
%   tx_baseband - 发射基带信号
%   rx_baseband - 接收基带信号 (各UAV)
%   pu_states - PU状态序列
%   params - 仿真参数

fprintf('正在生成可视化图表...\n');

% 时间向量
dt = 1 / params.fs;
t_rf = (0:length(tx_signal)-1)' * dt * 1e6; % 转换为微秒
t_baseband = t_rf; % 基带和射频时间相同

% 选择一个代表性的UAV进行显示 (SNR最好的)
rx_power = mean(abs(rx_baseband).^2, 1);
[~, best_uav_idx] = max(rx_power);

%% 1. 时域波形图
figure('Position', [100, 100, 1200, 800]);

% 限制显示时间范围 (前1000个样本)
max_samples = min(1000, length(tx_signal));
t_display = t_rf(1:max_samples);

subplot(2, 3, 1);
plot(t_display, real(tx_baseband(1:max_samples)), 'b-', 'LineWidth', 1.5);
xlabel('时间 (μs)');
ylabel('幅度');
title('PU发射基带信号 (实部)');
grid on;

subplot(2, 3, 2);
plot(t_display, tx_signal(1:max_samples), 'r-', 'LineWidth', 1);
xlabel('时间 (μs)');
ylabel('幅度');
title('PU发射射频信号');
grid on;

subplot(2, 3, 3);
plot(t_display, real(rx_baseband(1:max_samples, best_uav_idx)), 'g-', 'LineWidth', 1.5);
xlabel('时间 (μs)');
ylabel('幅度');
title(sprintf('UAV %d 接收基带信号 (实部)', best_uav_idx));
grid on;

subplot(2, 3, 4);
plot(t_display, imag(tx_baseband(1:max_samples)), 'b--', 'LineWidth', 1.5);
xlabel('时间 (μs)');
ylabel('幅度');
title('PU发射基带信号 (虚部)');
grid on;

subplot(2, 3, 5);
plot(t_display, rx_signals(1:max_samples, best_uav_idx), 'r-', 'LineWidth', 1);
xlabel('时间 (μs)');
ylabel('幅度');
title(sprintf('UAV %d 接收射频信号', best_uav_idx));
grid on;

subplot(2, 3, 6);
plot(t_display, imag(rx_baseband(1:max_samples, best_uav_idx)), 'g--', 'LineWidth', 1.5);
xlabel('时间 (μs)');
ylabel('幅度');
title(sprintf('UAV %d 接收基带信号 (虚部)', best_uav_idx));
grid on;

sgtitle('时域波形图', 'FontSize', 14, 'FontWeight', 'bold');

%% 2. 频域频谱图
figure('Position', [150, 150, 1200, 600]);

% FFT参数
N_fft = 2^nextpow2(length(tx_signal));
freq = (-N_fft/2:N_fft/2-1) * params.fs / N_fft / 1e6; % 转换为MHz

% 计算FFT
TX_spectrum = fftshift(fft(tx_signal, N_fft));
RX_spectrum = fftshift(fft(rx_signals(:, best_uav_idx), N_fft));
TX_baseband_spectrum = fftshift(fft(tx_baseband, N_fft));
RX_baseband_spectrum = fftshift(fft(rx_baseband(:, best_uav_idx), N_fft));

subplot(2, 2, 1);
plot(freq, 20*log10(abs(TX_spectrum) + eps), 'b-', 'LineWidth', 1.5);
xlabel('频率 (MHz)');
ylabel('幅度 (dB)');
title('PU发射射频信号频谱');
grid on;
xlim([params.fc/1e6 - 50, params.fc/1e6 + 50]); % 载波频率附近50MHz

subplot(2, 2, 2);
plot(freq, 20*log10(abs(RX_spectrum) + eps), 'r-', 'LineWidth', 1.5);
xlabel('频率 (MHz)');
ylabel('幅度 (dB)');
title(sprintf('UAV %d 接收射频信号频谱', best_uav_idx));
grid on;
xlim([params.fc/1e6 - 50, params.fc/1e6 + 50]);

subplot(2, 2, 3);
baseband_freq = freq - params.fc/1e6; % 基带频率
plot(baseband_freq, 20*log10(abs(TX_baseband_spectrum) + eps), 'b-', 'LineWidth', 1.5);
xlabel('频率 (MHz)');
ylabel('幅度 (dB)');
title('PU发射基带信号频谱');
grid on;
xlim([-params.bandwidth/1e6, params.bandwidth/1e6]);

subplot(2, 2, 4);
plot(baseband_freq, 20*log10(abs(RX_baseband_spectrum) + eps), 'g-', 'LineWidth', 1.5);
xlabel('频率 (MHz)');
ylabel('幅度 (dB)');
title(sprintf('UAV %d 接收基带信号频谱', best_uav_idx));
grid on;
xlim([-params.bandwidth/1e6, params.bandwidth/1e6]);

sgtitle('频域频谱图', 'FontSize', 14, 'FontWeight', 'bold');

%% 3. 功率谱密度图
figure('Position', [200, 200, 1200, 600]);

% 使用pwelch计算PSD
[psd_tx, freq_psd] = pwelch(tx_signal, [], [], N_fft, params.fs);
[psd_rx, ~] = pwelch(rx_signals(:, best_uav_idx), [], [], N_fft, params.fs);
[psd_tx_bb, freq_psd_bb] = pwelch(tx_baseband, [], [], N_fft, params.fs);
[psd_rx_bb, ~] = pwelch(rx_baseband(:, best_uav_idx), [], [], N_fft, params.fs);

freq_psd_mhz = freq_psd / 1e6;
freq_psd_bb_mhz = freq_psd_bb / 1e6 - params.fc / 1e6; % 基带频率

subplot(2, 2, 1);
plot(freq_psd_mhz, 10*log10(psd_tx + eps), 'b-', 'LineWidth', 1.5);
xlabel('频率 (MHz)');
ylabel('PSD (dB/Hz)');
title('PU发射射频信号PSD');
grid on;
xlim([params.fc/1e6 - 50, params.fc/1e6 + 50]);

subplot(2, 2, 2);
plot(freq_psd_mhz, 10*log10(psd_rx + eps), 'r-', 'LineWidth', 1.5);
xlabel('频率 (MHz)');
ylabel('PSD (dB/Hz)');
title(sprintf('UAV %d 接收射频信号PSD', best_uav_idx));
grid on;
xlim([params.fc/1e6 - 50, params.fc/1e6 + 50]);

subplot(2, 2, 3);
plot(freq_psd_bb_mhz, 10*log10(abs(psd_tx_bb) + eps), 'b-', 'LineWidth', 1.5);
xlabel('频率 (MHz)');
ylabel('PSD (dB/Hz)');
title('PU发射基带信号PSD');
grid on;
xlim([-params.bandwidth/1e6, params.bandwidth/1e6]);

subplot(2, 2, 4);
plot(freq_psd_bb_mhz, 10*log10(abs(psd_rx_bb) + eps), 'g-', 'LineWidth', 1.5);
xlabel('频率 (MHz)');
ylabel('PSD (dB/Hz)');
title(sprintf('UAV %d 接收基带信号PSD', best_uav_idx));
grid on;
xlim([-params.bandwidth/1e6, params.bandwidth/1e6]);

sgtitle('功率谱密度图', 'FontSize', 14, 'FontWeight', 'bold');

%% 4. PU状态时序图
figure('Position', [250, 250, 1000, 400]);

% 限制显示的时隙数量
max_slots = min(100, length(pu_states));
time_slots = 1:max_slots;

subplot(2, 1, 1);
stairs(time_slots, pu_states(1:max_slots), 'LineWidth', 2, 'Color', 'red');
xlabel('时间槽');
ylabel('PU状态');
title('PU活动状态时序图');
ylim([-0.1, 1.1]);
yticks([0, 1]);
yticklabels({'空闲', '活动'});
grid on;

% 统计信息
active_ratio = sum(pu_states) / length(pu_states) * 100;
text(max_slots*0.7, 0.8, sprintf('活动占比: %.1f%%', active_ratio), ...
    'FontSize', 12, 'BackgroundColor', 'white', 'EdgeColor', 'black');

subplot(2, 1, 2);
% 显示部分接收信号与PU状态的对应关系
samples_per_slot = params.oversample_factor;
signal_indices = 1:max_slots*samples_per_slot;
time_continuous = signal_indices / samples_per_slot;

plot(time_continuous, real(rx_baseband(signal_indices, best_uav_idx)), 'b-', 'LineWidth', 1);
hold on;

% 叠加PU状态
for i = 1:max_slots
    if pu_states(i) == 1
        fill([i-1, i, i, i-1], [-1, -1, 1, 1] * max(abs(rx_baseband(signal_indices, best_uav_idx))), ...
            'red', 'FaceAlpha', 0.2, 'EdgeColor', 'none');
    end
end

xlabel('时间槽');
ylabel('接收信号幅度');
title(sprintf('UAV %d 接收信号与PU状态对应关系', best_uav_idx));
grid on;
legend('接收信号', 'PU活动期间', 'Location', 'best');

%% 5. 星座图
figure('Position', [300, 300, 800, 400]);

% 下采样恢复符号
decimated_tx = tx_baseband(1:params.oversample_factor:end);
decimated_rx = rx_baseband(1:params.oversample_factor:end, best_uav_idx);

% 只显示PU活动期间的符号
active_symbols_tx = decimated_tx(pu_states == 1);
active_symbols_rx = decimated_rx(pu_states == 1);

% 限制显示的符号数量
max_symbols = min(500, length(active_symbols_tx));

subplot(1, 2, 1);
scatter(real(active_symbols_tx(1:max_symbols)), imag(active_symbols_tx(1:max_symbols)), ...
    20, 'filled', 'blue', 'MarkerFaceAlpha', 0.6);
xlabel('实部');
ylabel('虚部');
title('发射QPSK星座图');
grid on;
axis equal;

subplot(1, 2, 2);
scatter(real(active_symbols_rx(1:max_symbols)), imag(active_symbols_rx(1:max_symbols)), ...
    20, 'filled', 'red', 'MarkerFaceAlpha', 0.6);
xlabel('实部');
ylabel('虚部');
title(sprintf('UAV %d 接收QPSK星座图', best_uav_idx));
grid on;
axis equal;

sgtitle('QPSK调制星座图', 'FontSize', 14, 'FontWeight', 'bold');

fprintf('可视化图表生成完成！\n');
fprintf('  选择UAV %d 作为代表进行显示\n', best_uav_idx);
fprintf('  时域波形: 显示前 %d 个样本\n', max_samples);
fprintf('  PU状态: 显示前 %d 个时隙\n', max_slots);
fprintf('  星座图: 显示前 %d 个符号\n', max_symbols);
fprintf('\n');

end