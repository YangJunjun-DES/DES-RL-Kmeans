function optimal_action = choose_optimal_action(state_,Q_table)
    
    [~,b] = max(Q_table(state_, :));
%     if length(b) > 1
%         optimal_action = b(randperm(numel(b),1));
%     else
%         optimal_action = b;
%     end
    optimal_action = b;