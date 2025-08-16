function [pu_states, transition_matrix] = markov_pu_model(active_slots, idle_slots, total_time_slots)
%% PU二状态马尔可夫过程建模
% 输入:
%   active_slots - 平均活动时间槽数
%   idle_slots - 平均空闲时间槽数  
%   total_time_slots - 总仿真时间槽数
% 输出:
%   pu_states - PU状态序列 (0=空闲, 1=活动)
%   transition_matrix - 状态转移矩阵

%% 1. 计算状态转移概率
% 状态0: 空闲, 状态1: 活动
% 根据平均驻留时间计算转移概率

% P00: 从空闲状态保持空闲的概率
% P01: 从空闲状态转到活动的概率
% P10: 从活动状态转到空闲的概率  
% P11: 从活动状态保持活动的概率

% 平均驻留时间 = 1 / 转出概率
% 因此: P01 = 1 / idle_slots, P10 = 1 / active_slots
P01 = 1 / idle_slots;      % 空闲->活动
P10 = 1 / active_slots;    % 活动->空闲
P00 = 1 - P01;             % 空闲->空闲
P11 = 1 - P10;             % 活动->活动

% 状态转移矩阵
%        [空闲  活动]
% 空闲   [P00   P01]
% 活动   [P10   P11]
transition_matrix = [P00, P01; P10, P11];

fprintf('马尔可夫链参数:\n');
fprintf('  状态转移矩阵:\n');
fprintf('    P00 = %.3f, P01 = %.3f\n', P00, P01);
fprintf('    P10 = %.3f, P11 = %.3f\n', P10, P11);
fprintf('  平均空闲时间: %.1f 时隙\n', idle_slots);
fprintf('  平均活动时间: %.1f 时隙\n', active_slots);

%% 2. 计算稳态概率
% 求解 π = π * P, 其中π = [π0, π1]
% π0 + π1 = 1
% π0 * P00 + π1 * P10 = π0
% 解得: π0 = P10 / (P01 + P10), π1 = P01 / (P01 + P10)

pi0 = P10 / (P01 + P10);  % 空闲状态稳态概率
pi1 = P01 / (P01 + P10);  % 活动状态稳态概率

fprintf('  稳态概率: π0 = %.3f, π1 = %.3f\n', pi0, pi1);
fprintf('  理论活动占比: %.1f%%\n', pi1 * 100);

%% 3. 生成状态序列
pu_states = zeros(total_time_slots, 1);

% 初始状态根据稳态概率随机选择
if rand() < pi1
    pu_states(1) = 1;  % 活动
else
    pu_states(1) = 0;  % 空闲
end

% 根据转移概率生成后续状态
for t = 2:total_time_slots
    current_state = pu_states(t-1);
    
    if current_state == 0  % 当前空闲
        if rand() < P01
            pu_states(t) = 1;  % 转到活动
        else
            pu_states(t) = 0;  % 保持空闲
        end
    else  % 当前活动
        if rand() < P10
            pu_states(t) = 0;  % 转到空闲
        else
            pu_states(t) = 1;  % 保持活动
        end
    end
end

%% 4. 统计实际状态
actual_active_ratio = sum(pu_states) / total_time_slots;
state_changes = sum(diff(pu_states) ~= 0);

fprintf('  实际统计:\n');
fprintf('    活动占比: %.1f%%\n', actual_active_ratio * 100);
fprintf('    状态切换次数: %d\n', state_changes);

% 计算平均驻留时间
idle_segments = [];
active_segments = [];
current_segment_length = 1;
current_state = pu_states(1);

for t = 2:total_time_slots
    if pu_states(t) == current_state
        current_segment_length = current_segment_length + 1;
    else
        if current_state == 0
            idle_segments = [idle_segments; current_segment_length];
        else
            active_segments = [active_segments; current_segment_length];
        end
        current_state = pu_states(t);
        current_segment_length = 1;
    end
end

% 添加最后一个段
if current_state == 0
    idle_segments = [idle_segments; current_segment_length];
else
    active_segments = [active_segments; current_segment_length];
end

if ~isempty(idle_segments)
    fprintf('    实际平均空闲时间: %.1f 时隙\n', mean(idle_segments));
end
if ~isempty(active_segments)
    fprintf('    实际平均活动时间: %.1f 时隙\n', mean(active_segments));
end

fprintf('PU马尔可夫模型生成完成！\n\n');

end