function Index_up = Cluster_pattern(i, Q_converged, method)
%Use a cluster algorithm to generate a control pattern from the learned Q
%table

Value_state = Q_converged(i, :);
if method == 1 %Complex cluster
    if all(Value_state < 0)
        Index_up = 4;  %index=4 means no controllable is selected.
    else
        variance_threshold = 0.85; %设定阈值 (0:58/124; 0.1:58/124; 0.2:58/124; 0.3:58/124;0.4:58/124;0.5:58/124; 
                                          % 0.6:58/124; 0.7:58/126;
                                          % 0.8:58/131; 0.81:68/162;0.82:68/162; 0.83:68/162;0.84:68/165;
                                          % 0.85:72,178; 0.87:72,181; 
                                          % 0.88:72,182;0.89:deadlock;0.9:deadlock;)
                                          
                                           
                
        
        array_variance_norm = var_mapped(Value_state); %计算归一化后的数组方差
        
        if array_variance_norm < variance_threshold % 如果方差小于阈值，认为所有元素属于同一类
            % 获取所有下标
            Index_up = 1:length(Value_state);
        else  % 使用 kmeans 分为两类
            [idx, ~] = kmeans(Value_state', 2);
            mean1 = mean(Value_state(idx == 1));
            mean2 = mean(Value_state(idx == 2));
            if mean1 > mean2  %取较大值所属类别
                Index_up = find(idx == 1);
            else
                Index_up = find(idx == 2);
            end
        end
    end
 elseif method == 2  %Larger than zero or not
     if all(Value_state < 0)
        Index_up = 4;  %index=4 means no controllable is selected.
     else
        Index_up = find(Value_state > 0); 
     end
 end
% 默认两类，已验证，比原supervisor 少一个变迁，更新如上。    
%     %the indecies of the values larger than zero  分为两类
%     [idx, ~] = kmeans(Value_state', 2);
%     mean1 = mean(Value_state(idx == 1));
%     mean2 = mean(Value_state(idx == 2));
%     if mean1 > mean2
%         Index_up = find(idx == 1);
%     else
%         Index_up = find(idx == 2);
%     end
end







