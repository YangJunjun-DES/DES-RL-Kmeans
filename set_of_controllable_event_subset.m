function B = set_of_controllable_event_subset(E_c,k)
% Set of event subsets with at most k controllable events
        % 给定整数事件集合 E_c
    
    
    % 初始化存储子集的 cell 数组
    B = {[]};
    
    % 遍历 1 到 k，生成所有大小不超过 k 的子集
    for i = 1 : k
        combs = nchoosek(E_c, i); % 生成大小为 k 的所有子集
        B = [B; num2cell(combs, 2)]; % 存入 cell 数组，每个子集作为一个 cell
    end
    
    % 输出所有子集
%     disp('所有最多包含 k 个元素的子集：');
%     disp(B);
    
    


end