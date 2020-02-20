% ----------------------------------------------------------------
%% Post-Processing and Data Analysis
% ----------------------------------------------------------------


% ---------------------------------
%% Interface settings
% ---------------------------------
% set this to 1 to enable plots
enable_plots = 1;

% set this to 1 to enable docked window style in plots
enable_docked = 1;

if (enable_docked)
    set(0,'DefaultFigureWindowStyle','docked');
else    
    set(0,'DefaultFigureWindowStyle','normal');
end

set(0,'defaultAxesFontSize',14)
set(0,'DefaultLegendFontSize',14)


% ---------------------------------
%% Load vehicle data
% ---------------------------------
Lf = vehicle_data.vehicle.Lf;  % [m] Distance between vehicle CoG and front wheels axle
Lr = vehicle_data.vehicle.Lr;  % [m] Distance between vehicle CoG and front wheels axle
L  = vehicle_data.vehicle.L;   % [m] Vehicle length
Wf = vehicle_data.vehicle.Wf;  % [m] Width of front wheels axle 
Wr = vehicle_data.vehicle.Wr;  % [m] Width of rear wheels axle                   
m  = vehicle_data.vehicle.m;   % [kg] Vehicle Mass
g  = vehicle_data.vehicle.g;   % [m/s^2] Gravitational acceleration
tau_D = vehicle_data.steering_system.tau_D;  % [-] steering system ratio (pinion-rack)

         
% ---------------------------------
%% Extract data from simulink model
% ---------------------------------
time_sim = states.u.time;
dt = time_sim(2)-time_sim(1);

% -----------------
% Inputs
% -----------------
delta_D    = inputs.delta_D.data;
PID_torque = inputs.PID_torque.data; % torque provided by the PID controller
Tm_rr      = states.T_mot.data;      % right motor torque
Tm_rl      = states.T_mot.data;      % left motor torque
Tb_fr      = states.T_brake.data;    % front right wheel braking torque
Tb_fl      = states.T_brake.data;    % front left wheel braking torque
Tb_rr      = states.T_brake.data;    % rear right wheel braking torque
Tb_rl      = states.T_brake.data;    % rear left wheel braking torque

% -----------------
% States
% -----------------
u          = states.u.data;
v          = states.v.data;
z          = states.z.data; 
Omega      = states.Omega.data;
phi        = states.phi.data;
mu         = states.mu.data;
delta      = states.delta.data;
alpha_rr   = states.alpha_rr.data;
alpha_rl   = states.alpha_rl.data;
alpha_fr   = states.alpha_fr.data;
alpha_fl   = states.alpha_fl.data;
kappa_rr   = states.kappa_rr.data;
kappa_rl   = states.kappa_rl.data;
kappa_fr   = states.kappa_fr.data;
kappa_fl   = states.kappa_fl.data;
omega_rr   = states.omega_rr.data;
omega_rl   = states.omega_rl.data;
omega_fr   = states.omega_fr.data;
omega_fl   = states.omega_fl.data;
z_dot      = states.z_dot.data;
phi_dot    = states.phi_dot.data;
mu_dot     = states.mu_dot.data;

% -----------------
% Extra Parameters
% -----------------
Tw_rr      = extra_params.Tw_rr.data;
Tw_rl      = extra_params.Tw_rl.data;
Tw_fr      = extra_params.Tw_fr.data;
Tw_fl      = extra_params.Tw_fl.data;
Fx_rr      = extra_params.Fx_rr.data;
Fx_rl      = extra_params.Fx_rl.data;
Fx_fr      = extra_params.Fx_fr.data;
Fx_fl      = extra_params.Fx_fl.data;
Fy_rr      = extra_params.Fy_rr.data;
Fy_rl      = extra_params.Fy_rl.data;
Fy_fr      = extra_params.Fy_fr.data;
Fy_fl      = extra_params.Fy_fl.data;
Fz_rr      = extra_params.Fz_rr.data;
Fz_rl      = extra_params.Fz_rl.data;
Fz_fr      = extra_params.Fz_fr.data;
Fz_fl      = extra_params.Fz_fl.data;
Mz_rr      = extra_params.Mz_rr.data;
Mz_rl      = extra_params.Mz_rl.data;
Mz_fr      = extra_params.Mz_fr.data;
Mz_fl      = extra_params.Mz_fl.data;
gamma_rr   = extra_params.gamma_rr.data;
gamma_rl   = extra_params.gamma_rl.data;
gamma_fr   = extra_params.gamma_fr.data;
gamma_fl   = extra_params.gamma_fl.data;
delta_fr   = extra_params.delta_fr.data;
delta_fl   = extra_params.delta_fl.data;

% Chassis side slip angle beta [rad]
beta = atan(v./u);

% -----------------
% Accelerations
% -----------------
% Derivatives of u, v [m/s^2]
dot_u = diff(u)/dt;
dot_v = diff(v)/dt;
% Total longitudinal and lateral accelerations
Ax = dot_u - Omega(1:end-1).*v(1:end-1);
Ay = dot_v + Omega(1:end-1).*u(1:end-1);
% Ax low-pass filtered signal
[b_butt,a_butt] = butter(4,0.005,'low');
Ax_filt = filter(b_butt,a_butt,Ax);
% Longitudinal jerk [m/s^3]
jerk_x = diff(dot_u)/dt;
% Steady state lateral acceleration
Ay_ss = Omega.*u;

% -----------------
% Vehicle Pose
% -----------------
x_veh = vehicle_pose(:,1);
y_veh = vehicle_pose(:,2);
psi_veh = vehicle_pose(:,3);

% -----------------
% Vehicle route fitted with clothoids
% -----------------
vehRoute = ClothoidList();
numOfClothoids_route = size(refRoute_fewPoints,1);
for i = 1:numOfClothoids_route-1
    vehRoute.push_back_G1(refRoute_fewPoints(i,1),refRoute_fewPoints(i,2),refRoute_fewPoints(i,3), refRoute_fewPoints(i+1,1),refRoute_fewPoints(i+1,2),refRoute_fewPoints(i+1,3)); 
end

% -----------------
% Other parameters
% -----------------
% Total CoM speed [m/s]
vG = sqrt(u.^2 + v.^2);
% Steady state and transient curvature [m]
rho_ss   = Omega./vG;
rho_tran = ((dot_v.*u(1:end-1) - dot_u.*v(1:end-1)) ./ ((vG(1:end-1)).^3)) + rho_ss(1:end-1);
% Desired sinusoidal steering angle for the equivalent single track front wheel
desired_steer_atWheel = delta_D/tau_D;

% -----------------
% Data format manipulation
% -----------------
speed_request = zeros(size(speed_profile_req.data,3),1);
deltaD_request = zeros(size(delta_D,3),1);
for jj=1:size(speed_profile_req.data,3)
    speed_request(jj) = speed_profile_req.data(:,:,jj);
    deltaD_request(jj) = delta_D(:,:,jj);
end


% ---------------------------------
%% PLOTS
% ---------------------------------

if (enable_plots)
    % -------------------------------
    %% PLOT MAIN STATES
    % -------------------------------
    if (~enable_docked)
        figure('Name','States and controls','NumberTitle','off','Position',[0,0,500,1000]), clf 
        set(gcf,'units','points','position',[150,150,600,350])
    else
        figure('Name','States and controls','NumberTitle','off'), clf
    end    
    % --- u --- %
    ax(1) = subplot(221);
    plot(time_sim,u,'LineWidth',2)
    hold on
    plot(time_sim,speed_request,'--r','LineWidth',2)
    legend('real','desired','Location','best')
    grid on
    title('$u$ [m/s]')  
    xlim([0 time_sim(end)])
    % --- delta_D --- %
    ax(7) = subplot(222);
    plot(time_sim,rad2deg(deltaD_request),'LineWidth',1)
    hold on
    plot(time_sim,delta*tau_D,'Color',color('orange'),'LineWidth',2)
    grid on
    title('$\delta_D$ [deg]')
    legend('target','real')
    xlim([0 time_sim(end)])
    % --- Omega --- %
    ax(3) = subplot(223);
    plot(time_sim,Omega,'LineWidth',2)
    grid on
    title('$\Omega$ [rad/s]')
    xlim([0 time_sim(end)])

    % linkaxes(ax,'x')
    clear ax    

    % -------------------------------
    %% Plot vehicle pose x,y,psi
    % -------------------------------
    if (~enable_docked)
        figure('Name','Pose','NumberTitle','off','Position',[0,0,500,1000]), clf 
        set(gcf,'units','points','position',[150,150,600,350])
    else
        figure('Name','Pose','NumberTitle','off'), clf
    end 
    % --- x --- %
    ax(1) = subplot(221);
    plot(time_sim,x_veh,'LineWidth',2)
    grid on
    title('$x$ [m]')
    xlim([0 time_sim(end)])
    % --- y --- %
    ax(2) = subplot(222);
    plot(time_sim,y_veh,'LineWidth',2)
    grid on
    title('$y$ [m]')
    xlim([0 time_sim(end)])
    % --- psi --- %
    ax(3) = subplot(223);
    plot(time_sim,rad2deg(psi_veh),'LineWidth',2)
    grid on
    title('$\psi$ [deg]')
    xlim([0 time_sim(end)])

    % linkaxes(ax,'x')
    clear ax
    
    % -------------------------------
    %% Plot vehicle path
    % -------------------------------
    N = length(time_sim);
    [x_route,y_route] = vehRoute.evaluate(0:0.1:vehRoute.length);
    if (~enable_docked)
        figure('Name','Real Vehicle Path','NumberTitle','off','Position',[0,0,500,1000]), clf 
        set(gcf,'units','points','position',[150,150,800,280])
    else
        figure('Name','Real Vehicle Path','NumberTitle','off'), clf
    end 
    set(gca,'fontsize',16)
    hold on
    axis equal
    xlabel('x-coord [m]')
    ylabel('y-coord [m]')
    title('Scenario','FontSize',18)  %Real Vehicle Path
    % Plot road scenario
    for ii=1:size(mapObjects,1)
        rectangle('Position',mapObjects(ii,:),'FaceColor',color('purple'),'EdgeColor',color('purple'),'LineWidth',2)
    end
    %plot(costmap)
    % Plot interpolated reference route 
    plot(refRoute_fewPoints(:,1),refRoute_fewPoints(:,2),'go','MarkerFaceColor','g','MarkerSize',8) %,'DisplayName','Route'
    % Plot original (not interpolated) reference route 
    plot(refRoute_points_orig(:,1),refRoute_points_orig(:,2),'o','Color',color('orange'),'MarkerFaceColor',color('orange'),'MarkerSize',8) %,'DisplayName','Interpolated Route'
    % Plot vehicle CoM trajectory
    plot(x_veh,y_veh,'Color',color('gold'),'LineWidth',4)
    vehRoute.plot;
    for i = 1:floor(N/20):N
        rot_mat = [cos(psi_veh(i)) -sin(psi_veh(i)) ; sin(psi_veh(i)) cos(psi_veh(i))];
        pos_rr = rot_mat*[-Lr -Wr/2]';
        pos_rl = rot_mat*[-Lr +Wr/2]';
        pos_fr = rot_mat*[+Lf -Wf/2]';
        pos_fl = rot_mat*[+Lf +Wf/2]';
        pos = [pos_rr pos_rl pos_fl pos_fr];
        p = patch(x_veh(i) + pos(1,:),y_veh(i) + pos(2,:),'blue');
        quiver(x_veh(i), y_veh(i), u(i)*cos(psi_veh(i)), u(i)*sin(psi_veh(i)), 'color', [1,0,0]);
        quiver(x_veh(i), y_veh(i), -v(i)*sin(psi_veh(i)), v(i)*cos(psi_veh(i)), 'color', [0.23,0.37,0.17]);
    end
    plot(5,5,'mo','MarkerSize',6,'MarkerFaceColor','m')
    plot(196,134.5,'go','MarkerSize',6,'MarkerFaceColor','g')
    text(8,5,'A','FontSize',16);
    text(187,134.5,'B','FontSize',16);
    grid on
    legend('interpolating points','route','vehicle path','clothoid fitting','clothoid fitting','location','best')
    

end    


% -------------------------------
%% Compute the final time
% -------------------------------
parking_lot = mapObjects(1:2,:);
for ii=1:length(time_sim)
    if (x_veh(ii)>=parking_lot(1,1) && y_veh(ii)>=parking_lot(2,2) && y_veh(ii)<=parking_lot(1,2))
        final_time = ii*Ts;
        break;
    end
end
endOfCircuit_idx = ii;


% -------------------------------
%% Path tracking error
% -------------------------------
% Error between real and desired trajectories
desired_path = [x_route' y_route'];
% Prepare an array to store all the minimum distances between the points of
% the real and desired paths
pathTracking_error = zeros(size(x_veh));  % initialize

% For each point of the real vehicle path, find the point of the desired path
% that is closest to it, and the minimum distance between the 2 points
%for j = 1:length(x_veh)
for j = 1:endOfCircuit_idx
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
plot(pathTracking_error(1:endOfCircuit_idx),'-r','LineWidth',1.5)
xlabel('Path sample index [-]')
ylabel('Tracking error [m]')
title('Path tracking error','FontSize',18) 


% -------------------------------
%% Messages to display for the user
% -------------------------------
if (exist('final_time','var'))
    fprintf('The scenario is compled by the vehicle in %.3f seconds\n',final_time);
else
    fprintf('The vehicle did not manage to complete the scenario!\n');
end


