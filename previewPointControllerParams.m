function previewPointParams = previewPointControllerParams()

    % ----------------------------------------------------------------
    %% Function purpose: define the preview point lateral controller parameters
    % ----------------------------------------------------------------
    
    % Look ahead distance (in meters) used by the controller
    previewPointParams.lookAhead = 5; 
    %k1 and k2
    previewPointParams.k1 = 0.075;
    previewPointParams.k2 = 0.01;
end
