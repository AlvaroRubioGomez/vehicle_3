function initPose = loadInitialPose()
    
    %% ICs for the vehicle pose
    x0     = 5;     % [m] initial x-coordinate of the vehicle CoM
    y0     = 5;     % [m] initial y-coordinate of the vehicle CoM
    theta0 = 0;  % [rad] initial vehicle attitude
    
    initPose = [x0,y0,theta0]';
       
end

