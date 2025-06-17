function action = choose_action(current_state,Q_table,epsilon,Available_event)
tau = rand;
if tau > epsilon
    [~,b]=max(Q_table(current_state,Available_event));
    action = Available_event(b);
else
    action = Available_event(unidrnd(length(Available_event)));
    
end

