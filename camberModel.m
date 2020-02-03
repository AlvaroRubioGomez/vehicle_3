function [gamma__rr,gamma__rl,gamma__fr,gamma__fl] = camberModel(phi)

    % ----------------------------------------------------------------
    %% Function purpose: compute wheel camber values, depending on static camber
    %%                   and vehicle roll angle
    % ----------------------------------------------------------------
    
    % Load the struct containing vehicle parameters
    params = getVehicleDataStruct;
    camber_gain_const = params.suspension.camber_gain;
    rear_static_camber = params.rear_wheel.static_camber;
    front_static_camber = params.front_wheel.static_camber;

    % Wheel camber is composed of 2 main terms: static camber and camber
    % gain. Static camber is optimized depending on vehicle setup and it
    % can be easily varied, while camber gain is a function of suspension
    % design and it cannot be changed. In particular, camber gain can be
    % fitted from suspensions kinematic model, as a function of vehicle
    % roll angle phi
    camber_gain = camber_gain_const*(phi*180/pi);
    gamma__rr = (-rear_static_camber - camber_gain)*pi/180;    
    gamma__rl = (rear_static_camber - camber_gain)*pi/180;   
    gamma__fr = (-front_static_camber - camber_gain)*pi/180;
    gamma__fl = (front_static_camber - camber_gain)*pi/180;
    
end

