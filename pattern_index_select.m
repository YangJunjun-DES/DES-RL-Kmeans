function [max_value, max_value_indices, pattern_index] = pattern_index_select(Q_table, pattern_value, S, pattern_length, State_space)
    % Function to select the optimal pattern index based on Q_table evaluation
    % and the given pattern_value. If multiple indices have the same max value,
    % further evaluation is performed using Q_table.

    % 1. 获取 pattern_value 中的最大值
    max_value = max(pattern_value);
    
    % 2. 找出所有等于最大值的索引
    max_value_indices = find(pattern_value == max_value);
    
    % 3. 如果有多个索引有相同的最大值，需要进一步选择
    if length(max_value_indices) > 1
        % 从 Q_table 中获取当前状态 S 对应的所有动作的值
        [~,State] = ismember(S,State_space,"rows"); % state number
        pattern_value_Q = Q_table(State, :);
        
        % 在 Q_table 中，找到当前状态 S_norm 的最大值动作索引
        [~, pattern_index_Q] = max(pattern_value_Q);
        
        % 检查 pattern_index_Q 是否在 max_value_indices 中
        if any(max_value_indices == pattern_index_Q)
            pattern_index = pattern_index_Q;
        else
            % 如果没有 pattern_length, 则随机从 max_value_indices 中选择一个
            if nargin < 4 || isempty(pattern_length)
                pattern_index = max_value_indices(randi(length(max_value_indices)));
            else
                % 如果有 pattern_length, 选择具有最大长度的索引
                max_len = max(pattern_length(max_value_indices));
                max_len_indices = find(pattern_length(max_value_indices) == max_len);
                
                % 如果只有一个最长长度的索引, 选择它
                if length(max_len_indices) == 1
                    pattern_index = max_value_indices(max_len_indices);
                else
                    % 否则，从多个最长长度索引中随机选择一个
                    pattern_index = max_value_indices(randi(length(max_len_indices)));
                end
            end
        end
    elseif isempty(max_value_indices)
        error('Error: Unexpected condition, no max value indices found.');
    else
        % 如果只有一个最大值索引，则直接选择它
        pattern_index = max_value_indices(1);
    end
end