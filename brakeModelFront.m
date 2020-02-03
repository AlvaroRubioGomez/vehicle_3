function [Tw__fr,Tw__fl] = brakeModelFront(reqBrakeTorqueFR,reqBrakeTorqueFL,omega__fr,omega__fl)

    % ----------------------------------------------------------------
    %% Function purpose: compute braking torques at front wheels with brake model
    % ----------------------------------------------------------------

    params = getVehicleDataStruct;
    max_brake_torque_front = params.braking.max_brake_torque_front;
    regularSignScale = params.braking.regularSignScale;
    
    % Check that the braking torques have correctly been specified as
    % negative quantities. Otherwise, correct them by changing the sign
    if (reqBrakeTorqueFR > 0)
        reqBrakeTorqueFR = - reqBrakeTorqueFR;
    end
    
    if (reqBrakeTorqueFL > 0)
        reqBrakeTorqueFL = - reqBrakeTorqueFL;
    end
    
    % Check that the requested braking torque is lower than the one that
    % the hydraulic system can apply. 
    % Use regularized sign functions (sin(atan(.))) to make sure that the 
    % vehicle correctly stops at zero forward speed, to avoid that negative 
    % speed values could be reached during braking
    if (abs(reqBrakeTorqueFR) < abs(max_brake_torque_front))
        Tw__fr = reqBrakeTorqueFR*sin(atan(omega__fr/regularSignScale));
    else
        Tw__fr = -max_brake_torque_front*sin(atan(omega__fr/regularSignScale));
    end
    
    if (abs(reqBrakeTorqueFL) < abs(max_brake_torque_front))
        Tw__fl = reqBrakeTorqueFL*sin(atan(omega__fl/regularSignScale));
    else
        Tw__fl = -max_brake_torque_front*sin(atan(omega__fl/regularSignScale));
    end

end

