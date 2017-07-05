function T1 = invT(T0)

R = T0(1:3,1:3);
t = T0(1:3,4);
T1 = [R', -R'*t; 0 0 0 1];

end
