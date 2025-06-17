function Events = AvailableEvents(state, DFA)
Size = size(DFA);
Events = [];
for i = 1 : Size(1)
    for j = 1 : Size(3)
        if DFA(state,i,j) ~= 0
            Events(end+1) = j;
        end
    end
end
Events = unique(Events);

end

