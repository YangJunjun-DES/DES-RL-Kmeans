function array_variance = var_mapped(X)
    
    % 确保输入为列向量
    X = X(:);
    % 计算极差
    minX = min(X);
    maxX = max(X);
    rangeX = maxX - minX;
    % 处理全相同数据的情况
    if rangeX == 0
        array_variance = 0;
        return;
    end
    % 数据归一化（映射到[0,1]）
    X_normalized = (X - minX) / rangeX;
    
    % 计算归一化后的总体方差（除以n）
    var_normalized = var(X_normalized, 1);
    
    % 最终映射结果并保留两位小数
    array_variance = round(4 * var_normalized, 2); 
end

