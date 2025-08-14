%% UAV协作频谱感知分析
% 基于多UAV分布式感知结果的协作检测与决策融合
% 包含硬决策融合、软决策融合和性能比较

clear all; close all; clc;

%% 加载分布式感知结果
if exist('uav_distributed_sensing_results.mat', 'file')
    load('uav_distributed_sensing_results.mat');
    fprintf('=== UAV协作频谱感知分析 ===\n');
    fprintf('已加载分布式感知结果\n');
else
    fprintf('错误: 请先运行 uav_distributed_spectrum_sensing.m\n');
    return;
end

[N_uav, N_samples, N_time_slots] = size(received_signals);

%% 协作感知参数
% 检测阈值参数
false_alarm_rate = 0.1;  % 目标虚警率
threshold_factor = 2;    % 能量检测阈值因子

% 融合权重 (基于距离的权重)
max_distance = max(distances_3d);
distance_weights = (max_distance - distances_3d) / max_distance;
distance_weights = distance_weights / sum(distance_weights);

fprintf('\n协作感知参数:\n');
fprintf('  目标虚警率: %.2f\n', false_alarm_rate);
fprintf('  阈值因子: %.1f\n', threshold_factor);

%% 重新计算个体检测结果 (使用更精确的方法)
fprintf('\n重新计算个体检测结果...\n');

% 噪声功率估计 (使用PU空闲时的信号)
idle_slots = find(pu_state == 0);
noise_power_est = zeros(N_uav, 1);

for i = 1:N_uav
    if ~isempty(idle_slots)
        idle_signals = squeeze(received_signals(i, :, idle_slots));
        noise_power_est(i) = mean(var(idle_signals, 0, 1));
    else
        noise_power_est(i) = noise_power;
    end
end

% 计算个体检测统计量和判决
individual_test_stats = zeros(N_uav, N_time_slots);
individual_decisions = zeros(N_uav, N_time_slots);
individual_thresholds = zeros(N_uav, 1);

for i = 1:N_uav
    % 计算阈值 (基于噪声功率)
    individual_thresholds(i) = threshold_factor * noise_power_est(i);
    
    for t = 1:N_time_slots
        % 能量检测统计量
        signal = squeeze(received_signals(i, :, t));
        individual_test_stats(i, t) = mean(abs(signal).^2);
        
        % 个体判决
        individual_decisions(i, t) = (individual_test_stats(i, t) > individual_thresholds(i));
    end
end

%% 协作检测方法

% 1. 硬决策融合 (投票法)
fprintf('执行硬决策融合...\n');

% OR融合 (任一UAV检测到就判决为有信号)
hard_fusion_or = any(individual_decisions, 1);

% AND融合 (所有UAV都检测到才判决为有信号)
hard_fusion_and = all(individual_decisions, 1);

% 多数投票融合
vote_counts = sum(individual_decisions, 1);
hard_fusion_majority = (vote_counts > floor(N_uav/2));

% 2. 软决策融合 (基于检测统计量)
fprintf('执行软决策融合...\n');

% 等增益合并 (EGC)
soft_fusion_egc = mean(individual_test_stats, 1);
threshold_egc = mean(individual_thresholds);
soft_decisions_egc = (soft_fusion_egc > threshold_egc);

% 最大比合并 (MRC) - 基于SNR权重
snr_weights = zeros(N_uav, 1);
for i = 1:N_uav
    active_snr = snr_values(i, pu_state == 1);
    active_snr = active_snr(isfinite(active_snr));
    if ~isempty(active_snr)
        snr_weights(i) = mean(10.^(active_snr/10));
    else
        snr_weights(i) = 1;
    end
end
snr_weights = snr_weights / sum(snr_weights);

soft_fusion_mrc = zeros(1, N_time_slots);
for t = 1:N_time_slots
    soft_fusion_mrc(t) = sum(snr_weights .* individual_test_stats(:, t));
end
threshold_mrc = sum(snr_weights .* individual_thresholds);
soft_decisions_mrc = (soft_fusion_mrc > threshold_mrc);

% 基于距离的加权融合
soft_fusion_distance = zeros(1, N_time_slots);
for t = 1:N_time_slots
    soft_fusion_distance(t) = sum(distance_weights .* individual_test_stats(:, t));
end
threshold_distance = sum(distance_weights .* individual_thresholds);
soft_decisions_distance = (soft_fusion_distance > threshold_distance);

%% 性能评估
fprintf('\n=== 性能评估结果 ===\n');

% 真实标签
true_labels = pu_state;

% 计算性能指标的函数
calculate_performance = @(decisions) struct(...
    'accuracy', mean(decisions == true_labels), ...
    'detection_rate', sum(decisions & true_labels) / sum(true_labels), ...
    'false_alarm_rate', sum(decisions & ~true_labels) / sum(~true_labels), ...
    'missed_detection_rate', sum(~decisions & true_labels) / sum(true_labels));

% 各种方法的性能
perf_individual = zeros(N_uav, 1);
for i = 1:N_uav
    perf = calculate_performance(individual_decisions(i, :));
    perf_individual(i) = perf.accuracy;
    fprintf('UAV%d - 准确率: %.3f, 检测率: %.3f, 虚警率: %.3f\n', ...
        i, perf.accuracy, perf.detection_rate, perf.false_alarm_rate);
end

fprintf('\n硬决策融合性能:\n');
perf_or = calculate_performance(hard_fusion_or);
fprintf('OR融合 - 准确率: %.3f, 检测率: %.3f, 虚警率: %.3f\n', ...
    perf_or.accuracy, perf_or.detection_rate, perf_or.false_alarm_rate);

perf_and = calculate_performance(hard_fusion_and);
fprintf('AND融合 - 准确率: %.3f, 检测率: %.3f, 虚警率: %.3f\n', ...
    perf_and.accuracy, perf_and.detection_rate, perf_and.false_alarm_rate);

perf_majority = calculate_performance(hard_fusion_majority);
fprintf('多数投票 - 准确率: %.3f, 检测率: %.3f, 虚警率: %.3f\n', ...
    perf_majority.accuracy, perf_majority.detection_rate, perf_majority.false_alarm_rate);

fprintf('\n软决策融合性能:\n');
perf_egc = calculate_performance(soft_decisions_egc);
fprintf('EGC融合 - 准确率: %.3f, 检测率: %.3f, 虚警率: %.3f\n', ...
    perf_egc.accuracy, perf_egc.detection_rate, perf_egc.false_alarm_rate);

perf_mrc = calculate_performance(soft_decisions_mrc);
fprintf('MRC融合 - 准确率: %.3f, 检测率: %.3f, 虚警率: %.3f\n', ...
    perf_mrc.accuracy, perf_mrc.detection_rate, perf_mrc.false_alarm_rate);

perf_distance = calculate_performance(soft_decisions_distance);
fprintf('距离加权融合 - 准确率: %.3f, 检测率: %.3f, 虚警率: %.3f\n', ...
    perf_distance.accuracy, perf_distance.detection_rate, perf_distance.false_alarm_rate);

%% ROC曲线分析
fprintf('\n计算ROC曲线...\n');

% 生成不同阈值下的ROC点
threshold_range = linspace(0.5, 5, 50);
roc_individual = zeros(N_uav, length(threshold_range), 2); % [检测率, 虚警率]
roc_egc = zeros(length(threshold_range), 2);
roc_mrc = zeros(length(threshold_range), 2);

for idx = 1:length(threshold_range)
    thresh = threshold_range(idx);
    
    % 个体ROC
    for i = 1:N_uav
        thresh_scaled = thresh * noise_power_est(i);
        decisions = (individual_test_stats(i, :) > thresh_scaled);
        perf = calculate_performance(decisions);
        roc_individual(i, idx, :) = [perf.detection_rate, perf.false_alarm_rate];
    end
    
    % EGC ROC
    decisions_egc = (soft_fusion_egc > thresh * mean(noise_power_est));
    perf_egc_roc = calculate_performance(decisions_egc);
    roc_egc(idx, :) = [perf_egc_roc.detection_rate, perf_egc_roc.false_alarm_rate];
    
    % MRC ROC
    decisions_mrc = (soft_fusion_mrc > thresh * sum(snr_weights .* noise_power_est));
    perf_mrc_roc = calculate_performance(decisions_mrc);
    roc_mrc(idx, :) = [perf_mrc_roc.detection_rate, perf_mrc_roc.false_alarm_rate];
end

%% 绘制协作感知分析结果
fprintf('绘制协作感知分析图表...\n');

figure('Position', [100, 100, 1600, 1200]);

% 图1: 个体检测结果热图
subplot(3,4,1);
imagesc(individual_decisions);
colormap(gca, [1 1 1; 0 0 1]); % 白色=0, 蓝色=1
xlabel('时隙');
ylabel('UAV编号');
title('个体检测结果');
colorbar('Ticks', [0, 1], 'TickLabels', {'未检测', '检测到'});

% 图2: PU真实状态 vs 融合结果
subplot(3,4,2);
plot(1:N_time_slots, true_labels, 'k-', 'LineWidth', 2, 'DisplayName', '真实状态');
hold on;
plot(1:N_time_slots, hard_fusion_majority, 'r--', 'LineWidth', 1.5, 'DisplayName', '多数投票');
plot(1:N_time_slots, soft_decisions_egc, 'b:', 'LineWidth', 1.5, 'DisplayName', 'EGC融合');
xlabel('时隙');
ylabel('检测结果');
title('检测结果对比');
legend('Location', 'best');
ylim([-0.1, 1.1]);
grid on;

% 图3: 检测统计量时域图
subplot(3,4,3);
plot(1:N_time_slots, individual_test_stats(1:min(3,N_uav), :)', 'LineWidth', 1);
hold on;
plot(1:N_time_slots, soft_fusion_egc, 'k-', 'LineWidth', 2);
xlabel('时隙');
ylabel('检测统计量');
title('检测统计量时域');
legend([arrayfun(@(x) sprintf('UAV%d', x), 1:min(3,N_uav), 'UniformOutput', false), {'EGC融合'}]);
grid on;

% 图4: 权重分布
subplot(3,4,4);
bar([distance_weights, snr_weights], 'grouped');
xlabel('UAV编号');
ylabel('权重');
title('融合权重分布');
legend('距离权重', 'SNR权重');
grid on;

% 图5: 个体性能分布
subplot(3,4,5);
bar(1:N_uav, perf_individual);
xlabel('UAV编号');
ylabel('准确率');
title('个体检测准确率');
grid on;
ylim([0, 1]);

% 图6: 协作方法性能比较
subplot(3,4,6);
methods = {'OR', 'AND', '多数投票', 'EGC', 'MRC', '距离加权'};
accuracies = [perf_or.accuracy, perf_and.accuracy, perf_majority.accuracy, ...
              perf_egc.accuracy, perf_mrc.accuracy, perf_distance.accuracy];
bar(accuracies);
set(gca, 'XTickLabel', methods);
xtickangle(45);
ylabel('准确率');
title('协作方法性能比较');
grid on;
ylim([0, 1]);

% 图7: ROC曲线
subplot(3,4,7);
% 绘制个体UAV的ROC曲线 (选择前3个)
for i = 1:min(3, N_uav)
    plot(squeeze(roc_individual(i, :, 2)), squeeze(roc_individual(i, :, 1)), ...
        '--', 'LineWidth', 1, 'DisplayName', sprintf('UAV%d', i));
    hold on;
end
% 绘制协作方法的ROC曲线
plot(roc_egc(:, 2), roc_egc(:, 1), 'r-', 'LineWidth', 2, 'DisplayName', 'EGC融合');
plot(roc_mrc(:, 2), roc_mrc(:, 1), 'b-', 'LineWidth', 2, 'DisplayName', 'MRC融合');
plot([0, 1], [0, 1], 'k:', 'LineWidth', 1, 'DisplayName', '随机猜测');
xlabel('虚警率');
ylabel('检测率');
title('ROC曲线比较');
legend('Location', 'southeast');
grid on;
axis([0, 1, 0, 1]);

% 图8: 检测率 vs 虚警率
subplot(3,4,8);
detection_rates = [perf_or.detection_rate, perf_and.detection_rate, perf_majority.detection_rate, ...
                   perf_egc.detection_rate, perf_mrc.detection_rate, perf_distance.detection_rate];
false_alarm_rates = [perf_or.false_alarm_rate, perf_and.false_alarm_rate, perf_majority.false_alarm_rate, ...
                     perf_egc.false_alarm_rate, perf_mrc.false_alarm_rate, perf_distance.false_alarm_rate];
scatter(false_alarm_rates, detection_rates, 100, 'filled');
for i = 1:length(methods)
    text(false_alarm_rates(i)+0.01, detection_rates(i), methods{i}, 'FontSize', 8);
end
xlabel('虚警率');
ylabel('检测率');
title('检测性能散点图');
grid on;
axis([0, 1, 0, 1]);

% 图9: 距离对检测性能的影响
subplot(3,4,9);
scatter(distances_3d, perf_individual, 100, 'filled');
xlabel('距离 (m)');
ylabel('个体检测准确率');
title('距离 vs 检测性能');
grid on;

% 图10: SNR分布箱图
subplot(3,4,10);
active_slots = find(pu_state == 1);
if ~isempty(active_slots)
    snr_data = [];
    uav_labels = [];
    for i = 1:N_uav
        valid_snr = snr_values(i, active_slots);
        valid_snr = valid_snr(isfinite(valid_snr));
        if ~isempty(valid_snr)
            snr_data = [snr_data; valid_snr(:)];
            uav_labels = [uav_labels; i*ones(length(valid_snr), 1)];
        end
    end
    if ~isempty(snr_data)
        boxplot(snr_data, uav_labels);
        xlabel('UAV编号');
        ylabel('SNR (dB)');
        title('各UAV SNR分布');
        grid on;
    end
end

% 图11: 协作增益分析
subplot(3,4,11);
best_individual = max(perf_individual);
cooperation_gains = [perf_or.accuracy, perf_and.accuracy, perf_majority.accuracy, ...
                     perf_egc.accuracy, perf_mrc.accuracy, perf_distance.accuracy] - best_individual;
bar(cooperation_gains);
set(gca, 'XTickLabel', methods);
xtickangle(45);
ylabel('协作增益');
title('协作增益分析');
grid on;

% 图12: 时变性能分析
subplot(3,4,12);
window_size = 10;
sliding_accuracy = zeros(1, N_time_slots - window_size + 1);
for t = 1:(N_time_slots - window_size + 1)
    window_range = t:(t + window_size - 1);
    sliding_accuracy(t) = mean(soft_decisions_egc(window_range) == true_labels(window_range));
end
plot(1:length(sliding_accuracy), sliding_accuracy, 'b-', 'LineWidth', 2);
xlabel('时间窗口中心');
ylabel('滑动窗口准确率');
title('时变检测性能 (EGC)');
grid on;

sgtitle('UAV协作频谱感知性能分析');

%% 保存协作感知结果
save('uav_cooperative_sensing_results.mat', ...
    'individual_decisions', 'individual_test_stats', 'individual_thresholds', ...
    'hard_fusion_or', 'hard_fusion_and', 'hard_fusion_majority', ...
    'soft_decisions_egc', 'soft_decisions_mrc', 'soft_decisions_distance', ...
    'distance_weights', 'snr_weights', 'roc_individual', 'roc_egc', 'roc_mrc', ...
    'perf_individual', 'accuracies', 'detection_rates', 'false_alarm_rates');

fprintf('\n=== 协作感知分析完成 ===\n');
fprintf('最佳个体性能: %.3f\n', max(perf_individual));
fprintf('最佳协作性能: %.3f (方法: %s)\n', max(accuracies), methods{find(accuracies == max(accuracies), 1)});
fprintf('协作增益: %.3f\n', max(accuracies) - max(perf_individual));
fprintf('结果已保存到 uav_cooperative_sensing_results.mat\n');