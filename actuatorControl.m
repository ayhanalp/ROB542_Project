% defines pressure control signal dependent on state
function P = actuatorControl(t,x,a)
    
    Kp = 30;
    Kd = 5;
    
    bar = @(psi) 0.0689476 * psi;
    P_des = bar(100) * abs(sin(t*pi/2.5)); %periodic pressure; ea. 2sec
    
    dP_des = bar(100) * (pi/2.5) * abs(-cos(t*pi/2.5));
    
    P = Kp*(P_des - x(1)) + Kd*(dP_des - x(2));
    
    %P = bar(45);
end