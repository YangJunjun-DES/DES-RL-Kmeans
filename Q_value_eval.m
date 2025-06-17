function [pattern_value, pattern_length] = Q_value_eval(Q_table, S, R, E_c, E_u)

load("P1.mat");   load("P2.mat"); load("P3.mat"); %Components
load("R_B1SUP.mat");  load("R_B2SUP.mat"); %Supervisors
load("State_space.mat"); 



% 初始化 pattern_value 和 pattern_length，用于存储每个动作的评估值和评估长度
pattern_value = zeros(1, size(Q_table, 2));
pattern_length = zeros(1, size(Q_table, 2));

% 遍历所有可能的动作
for pattern_ind = 1:size(Q_table, 2)
    is_Done_test = 0;
    % 执行相应的 Step 函数
    % Components + Supervisors permit events + Agent
    [Event_set,Enb_P] = AllowedEvnts(S,P1,P2,P3,R_B1SUP, R_B2SUP);
    if pattern_ind ~= 4
        pattern = intersect(Event_set, E_c(pattern_ind));
        pattern = union(pattern, E_u);
        pattern = intersect(pattern, Enb_P);
    else
        pattern = setdiff(Event_set, E_c);
    end
    if ~isempty(pattern) % all_S, R_t and is_Done_test
        State_all = [];
        R_t = 0;
        for event_ind = 1 : length(pattern)
            S_ = StepFunction(P1,P2,P3,R_B1SUP,R_B2SUP,S,pattern(event_ind));
            [~,State_] = ismember(S_,State_space,"rows"); % next state number
            State_all = [State_all, State_];
            R_t = R_t + R(pattern(event_ind));
            [Event_set_,~] = AllowedEvnts(S_,P1,P2,P3,R_B1SUP, R_B2SUP);
            if isempty(Event_set_)
                is_Done_test = 1;
            end
        end
    else
        is_Done_test = 1;
    end

    if is_Done_test
        pattern_value(pattern_ind) = -1e5;
        pattern_length = 0;
    else
        Q_S_ = 0;
        for State_ind = 1 : length(State_all)
            Q_S_ = Q_S_ + max(Q_table(State_all(State_ind)));
        end
        Q_S_exp = Q_S_ ./ length(State_all);  % 使用均值而非最大值，因为有可能进入任何状态
        R_t = R_t ./ length(State_all);
        %  the expected value of Q is two step look ahead
        pattern_value(pattern_ind) = R_t + Q_S_exp;
        pattern_length(pattern_ind) = length(State_all);
    end
end








       