clear;clc;
load("P1.mat");   load("P2.mat"); load("P3.mat"); %Components
load("R_B1SUP.mat");  load("R_B2SUP.mat"); %Supervisors
load("Q_table.mat");
load("State_space.mat"); 

E = [1,2,3,4,5,6,8];
E_c = [1,3,5];
E_u = setdiff(E,E_c);
B = ff2n(length(E_c)); % controllable events distribution

num_transtions = 0;
OBSER = 1;
OBSER1 = 1;
while(~isempty(OBSER))
    state = OBSER(1);
    Observation = State_space(state,:);
    Enable_P1 = AvailableEvents(Observation(1), P1); 
    Enable_P2 = AvailableEvents(Observation(2), P2);
    Enable_P3 = AvailableEvents(Observation(3), P3);
    Enable_P = union(union(Enable_P1,Enable_P2),Enable_P3); %  Components available events at the current state
    Event_set_1 = AvailableEvents(Observation(4),R_B1SUP);  
    Event_set_2 = AvailableEvents(Observation(5),R_B2SUP);
    Event_set_12 = intersect(Event_set_1,Event_set_2); %  Supervisors available events at the current state
    
    
    
    
    %the learning result: Optimal pattern
    [max_q, index] = max(Q_table(state,:)); % choose the control pattern with the maximal value based on the obtained Q_table
   
    control_events = B(index,:);
    events_index = find(control_events == 1);
    pattern_control_c = E_c(events_index);   
    
    pattern = intersect(pattern_control_c,Event_set_12);
    pattern = union(pattern,E_u); 
    pattern = intersect(pattern,Enable_P);
    
    for i = 1 : length(pattern)
        event = pattern(i);
        observation_ = StepFunction(P1,P2,P3,R_B1SUP,R_B2SUP,Observation,event);
        [~,state_] = ismember(observation_,State_space,"rows");
        if (~ismember(state_,OBSER1)) 
            OBSER1(end+1) = state_;
            OBSER(end+1) = state_;       
        end
         
        
        fprintf('%d->%d[label=%d];\n',state,state_,event);
        num_transtions = num_transtions + 1;              
        
    end
    OBSER(1) = [];
end
fprintf('Transitions : %d\n',num_transtions);


% 3D fig
% d=zeros(28,8);
% for i =1: 28
%     for j = 1:8
%         d(i,j) = Q_table(OBSER1(i),j);
%     end
% end
% b = 1:8;
% c=1:28;
% [xx,yy]=meshgrid(b,c);
% plot3(xx,yy,d);
    