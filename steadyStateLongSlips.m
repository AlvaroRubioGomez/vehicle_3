function [kappa__rr_ss,kappa__rl_ss,kappa__fr_ss,kappa__fl_ss] = steadyStateLongSlips(omega__rr,omega__rl,omega__fr,omega__fl,Omega,u,delta__fr,delta__fl)

    % ----------------------------------------------------------------
    %% Function purpose: compute steady-state longitudinal slips
    % ----------------------------------------------------------------
    
    % Load the struct containing vehicle parameters
    params = getVehicleDataStruct;
    Rr     = params.rear_wheel.Rr;  
    Rf     = params.front_wheel.Rf; 
    Wr     = params.vehicle.Wr;  
    Wf     = params.vehicle.Wf;     
    Lf     = params.vehicle.Lf;        
    
    % kappa_rr
    if max(abs((omega__rr * Rr)),  abs((Omega * Wr) / 0.2e1 + u)) <= 4
        kappa__rr_ss = 2*((omega__rr * Rr) - ((Omega * Wr) / 0.2e1 + u)) / (4 + (max(abs((omega__rr * Rr)),  abs((Omega * Wr) / 0.2e1 + u)))^2 / 4);
    else
        kappa__rr_ss = ((omega__rr * Rr) - ((Omega * Wr) / 0.2e1 + u)) / max(abs((omega__rr * Rr)),  abs((Omega * Wr) / 0.2e1 + u));
    end

    % kappa_rl
    if max(abs((omega__rl * Rr)), abs(-(Omega * Wr) / 0.2e1 + u)) <= 4
        kappa__rl_ss = 2*((omega__rl * Rr) - (-(Omega * Wr) / 0.2e1 + u)) / (4 + (max(abs((omega__rl * Rr)), abs(-(Omega * Wr) / 0.2e1 + u)))^2 / 4);
    else
        kappa__rl_ss = ((omega__rl * Rr) - (-(Omega * Wr) / 0.2e1 + u)) / max(abs((omega__rl * Rr)), abs(-(Omega * Wr) / 0.2e1 + u));
    end

    % kappa_fr
    if max(abs((omega__fr * Rf)),  abs((Omega * Wf) / 0.2e1 + u + (delta__fr * Omega * Lf))) <= 4
        kappa__fr_ss = 2*((omega__fr * Rf) - ((Omega * Wf) / 0.2e1 + u + (delta__fr * Omega * Lf))) / (4 + (max(abs((omega__fr * Rf)),  abs((Omega * Wf) / 0.2e1 + u + (delta__fr * Omega * Lf))))^2 / 4);
    else
        kappa__fr_ss = ((omega__fr * Rf) - ((Omega * Wf) / 0.2e1 + u + (delta__fr * Omega * Lf))) / max(abs((omega__fr * Rf)),  abs((Omega * Wf) / 0.2e1 + u + (delta__fr * Omega * Lf)));
    end

    % kappa_fl
    if max(abs((omega__fl * Rf)), abs(-(Omega * Wf) / 0.2e1 + u + (delta__fl * Omega * Lf))) <= 4
        kappa__fl_ss = 2*((omega__fl * Rf) - (-(Omega * Wf) / 0.2e1 + u + (delta__fl * Omega * Lf))) / (4 + (max(abs((omega__fl * Rf)), abs(-(Omega * Wf) / 0.2e1 + u + (delta__fl * Omega * Lf))))^2 / 4 );
    else
        kappa__fl_ss = ((omega__fl * Rf) - (-(Omega * Wf) / 0.2e1 + u + (delta__fl * Omega * Lf))) / max(abs((omega__fl * Rf)), abs(-(Omega * Wf) / 0.2e1 + u + (delta__fl * Omega * Lf)));
    end
    
end

