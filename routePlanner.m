% ----------------------------------------------------
%% Route Planner
% ----------------------------------------------------

% ----------------------
% initial and final conditions
% ----------------------
x0 = initPose(1);       % [m]
y0 = initPose(2);       % [m]
theta0 = initPose(3);   % [rad]
startPose = [x0, y0, theta0]; 
goalPose  = loadFinalPose;

% ----------------------
% compute minimum turning radius
% ----------------------
L = vehicleDims.Wheelbase;
max_steer_angle = 4;  % [deg] max steering angle at the front wheel 
min_turn_radius = L/tan(deg2rad(max_steer_angle));


% ----------------------
% Solve the problem
% ----------------------
planner = pathPlannerRRT(costmap,'ConnectionDistance',15,...
        'GoalBias',0.1,'MinIterations',1e3,'MaxIterations',1e6,...
        'MinTurningRadius',min_turn_radius);
tic;
refPath = plan(planner,startPose,goalPose);
elapsed_time_routePlan = round(toc,1); 

% interpolate the calculated reference path
refRoute_points_orig = interpolate(refPath);
interp_vector_fewPoints = 0:50:refPath.Length;
refPath_poses_fewPoints = interpolate(refPath,interp_vector_fewPoints);

isPathValid = checkPathValidity(refPath,costmap);

% ----------------------
% Fit the interpolated path with clothoids
% ----------------------
path_fewPoints  = ClothoidList();
numOfClothoids_fewPoints = size(refPath_poses_fewPoints,1);
for jj = 1:numOfClothoids_fewPoints-1
    path_fewPoints.push_back_G1(refPath_poses_fewPoints(jj,1),refPath_poses_fewPoints(jj,2),deg2rad(refPath_poses_fewPoints(jj,3)), refPath_poses_fewPoints(jj+1,1),refPath_poses_fewPoints(jj+1,2),deg2rad(refPath_poses_fewPoints(jj+1,3))); 
end
% Compute the local curvature for the reference route
interp_vector_fewPoints = [interp_vector_fewPoints, path_fewPoints.length];    % add also the final route point
[x_cloth_refPath_fewPoints,y_cloth_refPath_fewPoints,theta_cloth_refPath_fewPoints,curv_refPath_fewPoints] = path_fewPoints.evaluate(interp_vector_fewPoints);
refRoute_fewPoints = [x_cloth_refPath_fewPoints',y_cloth_refPath_fewPoints',theta_cloth_refPath_fewPoints',curv_refPath_fewPoints'];


% ----------------------------------------------------
%% Save the resulting path
% ----------------------------------------------------
save('./refPath_poses_fewPoints','refRoute_fewPoints');
save('./refRoute_poses_RRT','refRoute_points_orig');
save('./cpuTime_routePlan','elapsed_time_routePlan');


% ----------------------------------------------------
%% Plot the resulting path
% ----------------------------------------------------
enable_plotRoute = 0;
if (enable_plotRoute)
    if (~enable_docked)
        figure('Name','Path RRT*','NumberTitle','off','Position',[0,0,500,1000]), clf 
        set(gcf,'units','points','position',[150,150,600,350])
    else
        figure('Name','Path RRT*','NumberTitle','off'), clf
    end   
    hold on
    %plot(planner)
    plot(costmap)
    plot(refPath,'DisplayName','Planned Path')
    scatter(refRoute_points_orig(:,1),refRoute_points_orig(:,2),[],'filled', ...
        'DisplayName','Transition Poses')
    scatter(refPath_poses_fewPoints(:,1),refPath_poses_fewPoints(:,2),'DisplayName','Interpolated Poses')
    grid on
    axis equal
    xlabel('x [m]')
    ylabel('y [m]')
    title('Path RRT*')
end


