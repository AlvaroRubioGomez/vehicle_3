% -------------------------------
%% Path tracking error
% -------------------------------
discretization_factor = 0.1;
[x_route,y_route] = vehRoute.evaluate(0:discretization_factor:vehRoute.length);
% Error between real and desired trajectories
desired_path = [x_route' y_route'];
% Prepare an array to store all the minimum distances between the points of
% the real and desired paths
pathTracking_error = zeros(size(x_veh));  % initialize

% For each point of the real vehicle path, find the point of the desired path
% that is closest to it, and the minimum distance between the 2 points
for j = 1:length(x_veh)
    actual_point = [x_veh(j,1) y_veh(j,1)];
    % Euclidean distance 
    distances_real_des = sqrt(sum(bsxfun(@minus, desired_path, actual_point).^2,2));
    % find the smallest distance and the corresponding index:
    min_dist_real_des = min(distances_real_des);
    pathTracking_error(j) = min_dist_real_des;
end

% statistical quantities of interest:
% - max value;
% - mean value;
% - std dev;
max_tracking_error = max(pathTracking_error);
mean_tracking_error = mean(pathTracking_error);
std_tracking_error  = std(pathTracking_error);

if (~enable_docked)
    figure('Name','Path tracking error','NumberTitle','off','Position',[0,0,500,1000]), clf 
    set(gcf,'units','points','position',[150,150,800,280])
else
    figure('Name','Path tracking error','NumberTitle','off'), clf
end 
plot(pathTracking_error,'-r','LineWidth',1.5)
xlabel('Path sample index [-]')
ylabel('Tracking error [m]')
title('Path tracking error','FontSize',18) 