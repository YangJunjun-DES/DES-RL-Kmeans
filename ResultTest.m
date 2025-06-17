

state = 1;
C_B1 = 0;
C_B2 = 0;
Policy_set = [];
generated_states = [1,1,1,1,1];
for step = 1:500
   
    [policy, next_state, state_] = Policy(state, Q_table, P1,P2,P3,B1SUP,B2SUP,State_space);
    if policy == 0
        fprintf('100-step failed!\n');
        break
    elseif policy == 2 || policy == 8
        C_B1 = C_B1 + 1;
    elseif policy == 3
        C_B1 = C_B1 - 1;
    elseif policy == 4
        C_B2 = C_B2 + 1;
    elseif policy == 5
        C_B2 = C_B2 - 1;
    end
    
    

    if C_B1 < 0 || C_B1 > 3 || C_B2 < 0 || C_B2 > 1
        fprintf('Buffer errors!\n');
        break
    end
    obs = State_space(state, :);
    if ~any(ismember(generated_states, obs_, 'rows'))
        generated_states = [generated_states; obs];
        fprintf('%d->%d[label=%d]\n', state, state_, policy);
    end 

    state = state_;
    Policy_set(end + 1) = policy;

end



