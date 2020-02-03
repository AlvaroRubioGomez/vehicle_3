classdef (StrictDefaults)PreviewPoint_lateralController < matlab.System & matlab.system.mixin.Propagates
    
    %% Public, tunable properties
    properties
        k1 = 1;
        k2 = 1;       
    end

%     %% Private properties
%     properties(Access = private)
%         % Initialize a clothoid for the vehicle path 
%         clothoid = ClothoidCurve();
%     end

    %% Methods
    methods(Access = protected)
        
    %% setupImpl - used for initialization
    function setupImpl(obj)
        
    end

    %% stepImpl - called at each time step
    function [delta_req] = stepImpl(obj,targetPoint,vehPose,vehSpeed)
        % Extract vehicle center of mass position
        x_vehCoM = vehPose(1);      % Current vehicle CoM x coord [m]
        y_vehCoM = vehPose(2);      % Current vehicle CoM y coord [m]
        theta_vehCoM = vehPose(3);  % Current vehicle attitude [rad]
        
        % Extract the parameters of the target point
        x_target = targetPoint(1);      % Target x coord [m]
        y_target = targetPoint(2);      % Target y coord [m]
        theta_target = targetPoint(3);  % Target vehicle attitude [rad]
        
%         % Build the clothoid from the current vehicle pose to the target point
%         obj.clothoid.build_G1(x_vehCoM,y_vehCoM,theta_vehCoM, x_target,y_target,theta_target);
%         initCurv = obj.clothoid.kappaBegin();  % initial clothoid curvature
        
        % Compute lateral path tracking error 
        [x_closest,y_closest,~,~] = obj.vehRoute.closestPoint(x_target,y_target);
        ep = sqrt((x_closest - x_target)^2, (y_closest - y_target)^2);
        if(y_vehCoM <= y_closest)
            ep = -ep; %depending on which side of the curve, apply sign
        end
        
        % Compute heading error
        theta_e = theta_target - theta_vehCoM;        
        
        % Compute the desired steering angle
        delta_req = obj.k1*ep + obj.k2*theta_e;
    end

    
    
    function [sz_1] = getOutputSizeImpl(~) 
        sz_1 = [1];
    end

    function [fz1] = isOutputFixedSizeImpl(~)
        fz1 = true;
    end

    function [dt1] = getOutputDataTypeImpl(~)
        dt1 = 'double';
    end

    function [cp1] = isOutputComplexImpl(~)
        cp1 = false;
    end
        
    end
end
