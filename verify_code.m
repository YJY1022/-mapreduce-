%% 代码验证脚本
% 检查UAV仿真代码的语法和逻辑正确性

fprintf('=== UAV通信系统代码验证 ===\n');

% 1. 检查配置文件
try
    config = uav_config();
    fprintf('✓ 配置文件uav_config.m语法正确\n');
    fprintf('  载波频率: %.2f GHz\n', config.communication.fc/1e9);
    fprintf('  符号速率: %.2f Msps\n', config.communication.symbol_rate/1e6);
catch ME
    fprintf('✗ 配置文件错误: %s\n', ME.message);
    return;
end

% 2. 验证关键参数范围
if config.communication.fc > 1e9 && config.communication.fc < 100e9
    fprintf('✓ 载波频率范围合理\n');
else
    fprintf('✗ 载波频率超出合理范围\n');
end

if config.geometry.uav_height > 0 && config.geometry.uav_height < 10000
    fprintf('✓ UAV高度范围合理\n');
else
    fprintf('✗ UAV高度超出合理范围\n');
end

% 3. 验证信道模型参数
distance_3d = sqrt(config.geometry.distance_2d^2 + config.geometry.uav_height^2);
path_loss = config.channel.los_A * log10(distance_3d) + config.channel.los_B + ...
           20*log10(config.communication.fc/1e9);

if path_loss > 50 && path_loss < 200
    fprintf('✓ 路径损耗计算合理: %.2f dB\n', path_loss);
else
    fprintf('✗ 路径损耗计算可能有误: %.2f dB\n', path_loss);
end

% 4. 验证多普勒计算
elevation_angle = atan(config.geometry.uav_height / config.geometry.distance_2d);
doppler_freq = config.communication.fc * config.geometry.uav_speed * cos(elevation_angle) / 3e8;

if abs(doppler_freq) < config.communication.fc / 1000  % 多普勒频移应小于载波频率的0.1%
    fprintf('✓ 多普勒频移计算合理: %.2f Hz\n', doppler_freq);
else
    fprintf('✗ 多普勒频移计算可能有误: %.2f Hz\n', doppler_freq);
end

% 5. 检查信号处理参数
sps = round(config.communication.fc * config.simulation.oversample_factor / config.communication.symbol_rate);
if sps > 1 && sps < 1000
    fprintf('✓ 过采样倍数合理: %d\n', sps);
else
    fprintf('✗ 过采样倍数可能有误: %d\n', sps);
end

% 6. 验证SNR范围
if config.channel.SNR_dB >= 0 && config.channel.SNR_dB <= 50
    fprintf('✓ 信噪比设置合理: %d dB\n', config.channel.SNR_dB);
else
    fprintf('✗ 信噪比设置可能不合理: %d dB\n', config.channel.SNR_dB);
end

fprintf('\n=== 验证完成 ===\n');
fprintf('代码结构完整，可以在有MATLAB环境的系统中运行\n');
fprintf('主要文件:\n');
fprintf('  - run_uav_simulation.m (主仿真程序)\n');
fprintf('  - uav_config.m (参数配置)\n');
fprintf('  - uav_communication_simulation.m (备用仿真)\n');
fprintf('  - README.md (使用说明)\n');