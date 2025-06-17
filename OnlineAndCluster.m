load("P1.mat");   load("P2.mat"); load("P3.mat"); %Components
load("R_B1SUP.mat");  load("R_B2SUP.mat"); %Supervisors
load("Q_table.mat");
load("State_space.mat");

Problem_state = [];
reach_states = [];
RO_nodes = 3;
RO_traces = 50;
RO_depth = 4;
RO_gamma = 0.95;
n_actions = 4;

R = 0.1 * zeros(1,8);

%
num_transtions = 0;
OBSER = 1;
OBSER1 = 1;
Imp_Q_table = zeros(size(Q_table, 1), size(Q_table, 2));
while(~isempty(OBSER))
    state = OBSER(1);
    isDone = 0;
    %the available events by SCT
    obs = State_space(state, :);
    [Event_set,Enb_P] = AllowedEvnts(obs,P1,P2,P3,R_B1SUP,R_B2SUP);
    %On-line predictive control to improve the learned policy
    [pattern_value, ~] = rollout_test(Q_table, state, RO_nodes, ...
        RO_traces, RO_depth, RO_gamma, n_actions, R, State_space);
    Imp_Q_table(state, :) = pattern_value;
    Index_up = Cluster_pattern(state, pattern_value, 1);

    %Compute pattern from the available events
    pattern =  pattern_cluster(Index_up, Event_set, Enb_P, E_u, E_c);

    %execute all events in the generated pattern
    for event_idx = 1 : length(pattern)
        event = pattern(event_idx);
        obs1_ = StepFunction(P1,P2,P3,R_B1SUP,R_B2SUP,obs,event);
        [~,state1_] = ismember(obs1_,State_space,"rows");
        [Event_set_,~] = AllowedEvnts(obs1_,P1,P2,P3,R_B1SUP,R_B2SUP);
        if isempty(Event_set_)
            fprintf("A deadlock occurs!\n");
            isDone = 1;
            break
        else
            if (~ismember(state1_,OBSER1)) 
                OBSER1(end+1) = state1_;
                OBSER(end+1) = state1_;  
            end
            fprintf('%d->%d[label=%d];\n',state,state1_,event);
            num_transtions = num_transtions + 1; 
        end

    end
    OBSER(1) = [];
    %break if a deadlock occurs
    if isDone == 1
        break
    end


    % move to the next state
    policy = pattern(unidrnd(length(pattern)));
    obs_ = StepFunction(P1, P2, P3, R_B1SUP, R_B2SUP, obs, policy);
    obs = obs_;
    [~,state_] = ismember(obs_,State_space,"rows");
    state = state_;
end























