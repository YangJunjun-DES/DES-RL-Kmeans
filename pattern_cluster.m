function pattern = pattern_cluster(index_set, Event_set, Enb_P, E_u, E_c)
%Compute the generated pattern from the clustered indecies
pattern = [];
if index_set == 4
    pattern = setdiff(Event_set, E_c);
else

    for i = 1:length(index_set)
        index = index_set(i);
        if index ~= 4
            pattern1 = intersect(Event_set, E_c(index));
            pattern1 = union(pattern1, E_u);
            pattern1 = intersect(pattern1, Enb_P);
            pattern = union(pattern, pattern1);
        end
    end


end