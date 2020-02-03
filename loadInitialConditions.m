function ICs = loadInitialConditions()

    % ----------------------------------------------------------------
    %% Function purpose: define the initial conditions for vehicle states
    % ----------------------------------------------------------------

    %% Load vehicle and auxiliary data
    vehicle_data = getVehicleDataStruct();
    
    Rf = vehicle_data.front_wheel.Rf;
    Rr = vehicle_data.rear_wheel.Rr;
    h__rc_f = vehicle_data.front_suspension.h_rc_f; 
    h__rc_r = vehicle_data.rear_suspension.h_rc_r; 
    Lr = vehicle_data.vehicle.Lr; 
    Lf = vehicle_data.vehicle.Lf;     
    z__static = h__rc_f + Lf * ((h__rc_r - h__rc_f)/(Lf+Lr));
   
    
    %% ICs for all the states
    u0         = 0.1/3.6;
    v0         = 0;           % [m/s] initial lateral velocity
    z0         = z__static;   % [m] initial condition for the internal dof z
    Omega0     = 0;           % [rad/s] initial yaw rate
    phi0       = 0;           % [rad] initial roll angle
    mu0        = -0.3*pi/180; % [rad] initial pitch angle
    delta0     = 0;           % [rad] initial steering angle
    alpha__rr0 = 0;           % [rad] initial slip angle for rear right wheel
    alpha__rl0 = 0;           % [rad] initial slip angle for rear left wheel
    alpha__fr0 = 0;           % [rad] initial slip angle for front right wheel
    alpha__fl0 = 0;           % [rad] initial slip angle for front left wheel
    kappa__rr0 = 0;           % [-] initial rear right wheel longitudinal slip
    kappa__rl0 = 0;           % [-] initial rear left wheel longitudinal slip
    kappa__fr0 = 0;           % [-] initial front right wheel longitudinal slip
    kappa__fl0 = 0;           % [-] initial front left wheel longitudinal slip
    omega__rr0 = u0/Rr;       % [rad/s] initial rotational speed of rear right wheel
    omega__rl0 = u0/Rr;       % [rad/s] initial rotational speed of rear left wheel
    omega__fr0 = u0/Rf;       % [rad/s] initial rotational speed of front right wheel
    omega__fl0 = u0/Rf;       % [rad/s] initial rotational speed of front left wheel
    z_dot0     = 0;           % [m/s] initial condition for the derivative of the internal dof z_dot
    phi_dot0   = 0;           % [rad/s] initial speed of variation of roll angle
    mu_dot0    = 0;           % [rad/s] initial speed of variation of pitch angle
    Tm0        = 0;           % [Nm] initial motor torque
    Tb0        = 0;           % [Nm] initial brake torque
    
    ICs = [u0, v0, z0, Omega0, phi0, mu0, delta0, alpha__rr0, alpha__rl0, alpha__fr0, alpha__fl0, ...
           kappa__rr0, kappa__rl0, kappa__fr0, kappa__fl0, omega__rr0, omega__rl0, ...
           omega__fr0, omega__fl0, z_dot0, phi_dot0, mu_dot0, Tm0, Tb0];
       
end

