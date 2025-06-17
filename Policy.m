function [policy, next_state,i_] = Policy(i, Q_converged, P1,P2,P3,B1SUP,B2SUP,Com)

E_c = [1,3,5];
E_u = [2,4,6,8];
    
    
    state = Com(i, :);
    Observation = state;
    
    % Components + Supervisors permit events
    [~, index] = max(Q_converged(i,:));
    [Event_set,Enb_P] = AllowedEvnts(Observation,P1,P2,P3,B1SUP,B2SUP);
    if index ~= 4
        pattern = intersect(Event_set, E_c(index));
        pattern = union(pattern, E_u);
        pattern = intersect(pattern, Enb_P);
    else
        pattern = setdiff(Event_set, E_c);
    end

    
    policy = pattern(unidrnd(length(pattern)));

    if ~isempty(policy)
        X1 = state(1);
        X2 = state(2);
        X3 = state(3);
        X4 = state(4);
        X5 = state(5);
    
        action = policy;
        X1_ = find(P1(X1,:,action) ~= 0);
        if isempty(X1_)
            X1_ = X1;
        end
    
        X2_ = find(P2(X2,:,action) ~= 0);
        if isempty(X2_)
            X2_ = X2;
        end
    
        X3_ = find(P3(X3,:,action) ~= 0);
        if isempty(X3_)
            X3_ = X3;
        end
        X4_ = find(B1SUP(X4,:,action) ~= 0);
        X5_ = find(B2SUP(X5,:,action) ~= 0);
        
        next_state = [X1_,X2_,X3_,X4_,X5_];
        [~,i_] = ismember(next_state,Com,"rows");
    
    %     for j = 1: length(Com)
    %        k = Com{j};
    %        if isequal(next_state,k)
    %            i_ = j;
    %        end
    %     end
    else
        policy = 0;
        next_state = state;
        i_ = 0;
    end


    
    
    

end

