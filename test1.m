clear;clc
tic
load("P1.mat");   load("P2.mat"); load("P3.mat"); %Components
load("R_B1SUP.mat");  load("R_B2SUP.mat"); %Supervisors
load("State_space.mat"); 
%load("Q_table.mat");
 

E_u = [2,4,6,8];
E_c = [1,3,5];


A = 1:1:length(E_c); %action set
Q_table = zeros(length(State_space),length(A)); % Q table

%reward of event
R = zeros(1,8);
for i = 1:8
    if i == 6
        R(i)=30;
    else
        R(i)=0.1;
    end
end

% parameters
epsilon = 0.98;
epsilonDecay = 1e-2;
epsilonMin = 0.05;
gamma = 0.96;
alpha = 0.9;

minimal_episode = 2000;
minimal_step = 100;
av = 0;
FiveEpisodeRewardTotal = zeros(1,100); %circle buffer

%update
episode = 0;

% episode < minimal_episode
while(episode < minimal_episode)  %5.7s
    num_i = 1;
    initial_Observation = State_space(num_i, :);
    Observation = initial_Observation;
    step = 0;
    EpisodeReward = 0;
    while(step<minimal_step)
        [~,state] = ismember(Observation,State_space,"rows"); % state number
        % select a control pattern by epsilon_greedy policy
        tau = rand;
        if tau > epsilon            
            [~,b]=max(Q_table(state,:));  
            index = b(unidrnd(length(b)));
        else
            index = A(unidrnd(length(A)));
        end
        Controllable_event = E_c(index);  %Selected controllable event
        % Components + Supervisors permit events + Agent
        [Event_set,Enb_P] = AllowedEvnts(Observation,P1,P2,P3,R_B1SUP, R_B2SUP);
        Uncontrollable_event = intersect(Enb_P, E_u);  %defined uncontrollable event
        pattern = union(Controllable_event, Uncontrollable_event);
        pattern = intersect(Event_set, pattern); 
        
                
        if ~isempty(pattern) && all(ismember(pattern, Event_set))
            %randomly choose an event form the selected available events
            Action = pattern(unidrnd(length(pattern)));  
            %execute the selected action and observe the next state    
            Observation_ = StepFunction(P1,P2,P3,R_B1SUP,R_B2SUP,Observation,Action);       
            [Event_set_,~] = AllowedEvnts(Observation_,P1,P2,P3,R_B1SUP,R_B2SUP);
            Sum_Max_value = 0; 
            Sum_reward = 0;       
            for i = 1:length(pattern)
                event = pattern(i);
                Next_obs = StepFunction(P1,P2,P3,R_B1SUP,R_B2SUP,Observation,event);  %One possible observation
                [Event_set_,~] = AllowedEvnts(Next_obs,P1,P2,P3,R_B1SUP, R_B2SUP);
                [~,Next_state] = ismember(Next_obs,State_space,"rows");
                Max_value = max(Q_table(Next_state,:));
                Reward = R(event);
                Sum_Max_value = Sum_Max_value + Max_value; %reward of all available transitions
                Sum_reward = Sum_reward + Reward;
            end   
            Sum_reward = Sum_reward / length(pattern);  
            Sum_Max_value = Sum_Max_value / length(pattern);
        else
            Event_set_ = [];  
            Sum_reward = -50;       
        end
                       
        % Deadlock or not
        termination = 0;
        if isempty(Event_set_)
            termination = 1;
        end
        
        %reward of the selected control pattern                        
        if termination
            target = -50;  
        else           
            target = Sum_reward + gamma * Sum_Max_value;
        end

        delta_Q = target - Q_table(state,index);
        Q_table(state,index) = Q_table(state,index) + alpha * delta_Q;
        step = step + 1;
        if epsilon > epsilonMin
            epsilon = epsilon * (1 - epsilonDecay);
        end
        EpisodeReward = EpisodeReward + Sum_reward; 
        %stop criteria
        if termination == 1
            break
        end        
        % Move to the next state if not terminated
        Observation = Observation_;
    end

    episode = episode + 1; 
    EpisodeRewardMemory(episode) = EpisodeReward;
    FiveEpisodeRewardTotal(end + 1) = EpisodeReward;

    if length(FiveEpisodeRewardTotal) > 100
        FiveEpisodeRewardTotal(1) = [];
    end

    AverageReward(episode) = mean(FiveEpisodeRewardTotal);
    av = AverageReward(episode);   
end
plot(1:episode, EpisodeRewardMemory); 
hold on
plot(1:episode, AverageReward); 

toc   






       





















