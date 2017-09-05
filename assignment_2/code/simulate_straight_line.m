%% Task 2.3


% Without current
t_end = 419;
U = 1.5;
w = 0.859;
t = 0:t_end;
v = zeros(t_end, 3);
s = zeros(t_end, 3);
for it = 0:t_end
    v(it+1,:) = [U*cosd(w*it) U*sind(w*it) 0];
    if it == 0
        s(it+1,1) = 0;
        s(it+1,2) = 0;
        s(it+1,3) = 0;
    else
        s(it+1,1) = s(it,1) + v(it,1);
        s(it+1,2) = s(it,2) + v(it,2);
        s(it+1,3) = s(it,3) + v(it,3);
    end
end
    
figure(1)
plot(t, s);

% With current

t_end = 419;
U = 1.5;
w = 0.859;
t = 0:t_end;
v = zeros(t_end, 3);
s = zeros(t_end, 3);
for it = 0:t_end
    v(it+1,:) = [0.926*cosd(w*it)-0.158*sind(w*it) 0.926*sind(w*it)+0.158*cosd(w*it) -0.074];
    if it == 0
        s(it+1,1) = 0;
        s(it+1,2) = 0;
        s(it+1,3) = 0;
    else
        s(it+1,1) = s(it,1) + v(it,1);
        s(it+1,2) = s(it,2) + v(it,2);
        s(it+1,3) = s(it,3) + v(it,3);
    end
end
    
figure(2)
plot(t, s);
