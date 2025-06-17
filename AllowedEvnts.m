function [Enb,Enable_P] = AllowedEvnts(Observation,P1,P2,P3,B1SUP,B2SUP)

    
        
    
    
    Enable_P1 = AvailableEvents(Observation(1), P1); 
    Enable_P2 = AvailableEvents(Observation(2), P2);
    Enable_P3 = AvailableEvents(Observation(3), P3);
    Enable_P = union(union(Enable_P1,Enable_P2),Enable_P3); %  Components available events at the current state
    Event_set_1 = AvailableEvents(Observation(4),B1SUP);  
    Event_set_2 = AvailableEvents(Observation(5),B2SUP);
    Event_set_12 = intersect(Event_set_1,Event_set_2);
    
    Enb = intersect(Enable_P,Event_set_12);  

















end

