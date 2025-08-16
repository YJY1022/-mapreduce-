function [channel_gains, los_probability, path_loss_db] = a2g_channel_model(distances, elevation_angles, fc, params)
%% A2G (Air-to-Ground) 信道模型
% 输入:
%   distances - UAV到PU的距离 (m)
%   elevation_angles - 仰角 (度)
%   fc - 载波频率 (Hz)
%   params - 包含信道参数的结构体
% 输出:
%   channel_gains - 复数信道增益
%   los_probability - LoS概率
%   path_loss_db - 路径损耗 (dB)

c = 3e8; % 光速
lambda = c / fc; % 波长

%% 1. 计算LoS概率 (Al-Hourani模型)
% P_LoS = 1 / (1 + a * exp(-b * (theta - a)))
los_probability = 1 ./ (1 + params.a * exp(-params.b * (elevation_angles - params.a)));

%% 2. 计算路径损耗
% 自由空间路径损耗 (dB)
fspl_db = 20 * log10(distances) + 20 * log10(fc) - 147.55;

% LoS和NLoS路径损耗
los_path_loss_db = fspl_db + 20 * log10(distances) * (params.los_path_loss_exp - 2) / 10;
nlos_path_loss_db = fspl_db + 20 * log10(distances) * (params.nlos_path_loss_exp - 2) / 10;

% 额外的NLoS损耗 (典型值20dB)
nlos_excess_loss_db = 20;
nlos_path_loss_db = nlos_path_loss_db + nlos_excess_loss_db;

%% 3. 根据LoS概率选择路径损耗
num_uav = length(distances);
path_loss_db = zeros(num_uav, 1);
is_los = rand(num_uav, 1) < los_probability; % 随机确定是否为LoS

for i = 1:num_uav
    if is_los(i)
        path_loss_db(i) = los_path_loss_db(i);
    else
        path_loss_db(i) = nlos_path_loss_db(i);
    end
end

%% 4. 小尺度衰落
small_scale_fading = zeros(num_uav, 1);

for i = 1:num_uav
    if is_los(i)
        % LoS: Nakagami-m衰落 (m=2, 等效于Rician K=1)
        m = 2; % Nakagami参数
        % Nakagami-m分布的幅度
        small_scale_fading(i) = sqrt(gamrnd(m, 1/m)) * exp(1j * 2 * pi * rand());
    else
        % NLoS: Rayleigh衰落
        % Rayleigh分布的复数形式
        real_part = randn() / sqrt(2);
        imag_part = randn() / sqrt(2);
        small_scale_fading(i) = real_part + 1j * imag_part;
    end
end

%% 5. 计算总的信道增益
% 路径损耗 (线性尺度)
path_loss_linear = 10.^(-path_loss_db / 10);

% 总信道增益 = sqrt(路径损耗) * 小尺度衰落
channel_gains = sqrt(path_loss_linear) .* small_scale_fading;

%% 6. 显示信道信息
fprintf('信道模型计算完成:\n');
fprintf('  LoS链路数量: %d / %d\n', sum(is_los), num_uav);
fprintf('  平均路径损耗: %.1f dB\n', mean(path_loss_db));
fprintf('  路径损耗范围: %.1f - %.1f dB\n', min(path_loss_db), max(path_loss_db));
fprintf('  平均信道增益: %.2e\n', mean(abs(channel_gains)));

end