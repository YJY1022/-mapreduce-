function [snr_db, signal_power_dbm, noise_power_dbm] = snr_calculation(tx_power_dbm, channel_gains, params)
%% SNR计算和噪声建模
% 输入:
%   tx_power_dbm - 发射功率 (dBm)
%   channel_gains - 信道增益 (复数)
%   params - 仿真参数结构体
% 输出:
%   snr_db - 信噪比 (dB)
%   signal_power_dbm - 接收信号功率 (dBm)
%   noise_power_dbm - 噪声功率 (dBm)

num_uav = length(channel_gains);

%% 1. 计算噪声功率
% 热噪声功率 = kTB (k: 玻尔兹曼常数, T: 温度, B: 带宽)
% 标准温度 T = 290K
k_boltzmann = 1.38e-23; % 玻尔兹曼常数 (J/K)
T_kelvin = 290;         % 标准温度 (K)

% 热噪声功率密度 (W/Hz)
thermal_noise_psd = k_boltzmann * T_kelvin;

% 热噪声功率密度 (dBm/Hz)
thermal_noise_psd_dbm = 10 * log10(thermal_noise_psd * 1000);

fprintf('噪声计算:\n');
fprintf('  理论热噪声PSD: %.1f dBm/Hz\n', thermal_noise_psd_dbm);
fprintf('  仿真噪声PSD: %.1f dBm/Hz\n', params.noise_psd_dbm_hz);

% 总噪声功率 = 噪声PSD + 10*log10(带宽) + 噪声因子
noise_power_dbm = params.noise_psd_dbm_hz + 10*log10(params.bandwidth) + params.noise_figure_db;

fprintf('  系统带宽: %.1f MHz\n', params.bandwidth/1e6);
fprintf('  噪声因子: %.1f dB\n', params.noise_figure_db);
fprintf('  总噪声功率: %.1f dBm\n', noise_power_dbm);

%% 2. 计算接收信号功率
% 接收功率 = 发射功率 + 信道增益(dB)
channel_gain_db = 20 * log10(abs(channel_gains)); % 转换为dB
signal_power_dbm = tx_power_dbm + channel_gain_db;

fprintf('\n接收信号功率:\n');
for i = 1:num_uav
    fprintf('  UAV %d: %.1f dBm (信道增益: %.1f dB)\n', i, signal_power_dbm(i), channel_gain_db(i));
end

%% 3. 计算SNR
snr_db = signal_power_dbm - noise_power_dbm;

fprintf('\nSNR计算结果:\n');
fprintf('  SNR范围: %.1f - %.1f dB\n', min(snr_db), max(snr_db));
fprintf('  平均SNR: %.1f dB\n', mean(snr_db));

% 统计SNR分布
snr_ranges = [-inf, -10, -5, 0, 5, 10, 15, 20, inf];
snr_labels = {'< -10', '-10~-5', '-5~0', '0~5', '5~10', '10~15', '15~20', '> 20'};

fprintf('\nSNR分布统计:\n');
for i = 1:length(snr_labels)
    if i == 1
        count = sum(snr_db < snr_ranges(i+1));
    elseif i == length(snr_labels)
        count = sum(snr_db >= snr_ranges(i));
    else
        count = sum(snr_db >= snr_ranges(i) & snr_db < snr_ranges(i+1));
    end
    percentage = count / num_uav * 100;
    fprintf('  %s dB: %d UAV (%.1f%%)\n', snr_labels{i}, count, percentage);
end

%% 4. 计算理论性能指标
fprintf('\n理论性能分析:\n');

% QPSK在AWGN信道下的误码率
% BER = Q(sqrt(2*SNR_linear))
snr_linear = 10.^(snr_db/10);
ber_theoretical = qfunc(sqrt(2 * snr_linear));

fprintf('  理论BER范围: %.2e - %.2e\n', min(ber_theoretical), max(ber_theoretical));
fprintf('  平均理论BER: %.2e\n', mean(ber_theoretical));

% 香农容量
% C = B * log2(1 + SNR_linear) (bps)
shannon_capacity = params.bandwidth * log2(1 + snr_linear);

fprintf('  香农容量范围: %.1f - %.1f Mbps\n', min(shannon_capacity)/1e6, max(shannon_capacity)/1e6);
fprintf('  平均香农容量: %.1f Mbps\n', mean(shannon_capacity)/1e6);

%% 5. 绘制SNR分布图
figure('Position', [200, 200, 1000, 400]);

subplot(1, 2, 1);
histogram(snr_db, 15, 'FaceColor', 'skyblue', 'EdgeColor', 'black');
xlabel('SNR (dB)');
ylabel('UAV数量');
title('SNR分布直方图');
grid on;
hold on;
xline(mean(snr_db), 'r--', 'LineWidth', 2, 'Label', sprintf('平均SNR: %.1f dB', mean(snr_db)));

subplot(1, 2, 2);
stem(1:num_uav, snr_db, 'filled', 'MarkerSize', 6);
xlabel('UAV索引');
ylabel('SNR (dB)');
title('各UAV的SNR');
grid on;
hold on;
yline(mean(snr_db), 'r--', 'LineWidth', 2, 'Label', sprintf('平均SNR: %.1f dB', mean(snr_db)));
legend('Location', 'best');

fprintf('\nSNR计算完成！\n\n');

end