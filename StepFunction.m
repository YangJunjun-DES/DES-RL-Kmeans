function Observation_ = StepFunction(P1,P2,P3,B1SUP,B2SUP,Observation,Action)
    X1 = Observation(1);
    X2 = Observation(2);
    X3 = Observation(3);
    X4 = Observation(4);
    X5 = Observation(5);

    X1_ = find(P1(X1,:,Action) ~= 0);
    if isempty(X1_)
        X1_ = X1;
    end

    X2_ = find(P2(X2,:,Action) ~= 0);
    if isempty(X2_)
        X2_ = X2;
    end

    X3_ = find(P3(X3,:,Action) ~= 0);
    if isempty(X3_)
        X3_ = X3;
    end
    X4_ = find(B1SUP(X4,:,Action) ~= 0);
    X5_ = find(B2SUP(X5,:,Action) ~= 0);

    Observation_ = [X1_,X2_,X3_,X4_,X5_];

end

