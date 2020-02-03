function clothoidBasedParams = clothoidBasedControllerParams()

    % ----------------------------------------------------------------
    %% Function purpose: define the clothoid-based lateral controller parameters
    % ----------------------------------------------------------------
    
    % Look ahead distance (in meters) used by the controller
    clothoidBasedParams.lookAhead = 10; 
    
    % Understeering gradient
    clothoidBasedParams.Kus = -8*10^(-4); 

end

