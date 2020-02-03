% ----------------------------------------------------------------
%% Main script for self-driving vehicle control framework 
%  authors: Mattia Piccinini & Francesco Biral
%  emails:  mattia.piccinini@unitn.it, francesco.biral@unitn.it
%  date:    10/12/2019
% ----------------------------------------------------------------

%% Initialize
clc
% clear all
% close all
clearvars
clear functions
format long;

% set this to 1 to enable docked window style in plots
enable_docked = 1;
if (enable_docked)
    set(0,'DefaultFigureWindowStyle','docked');
else    
    set(0,'DefaultFigureWindowStyle','normal');
end
set(0,'defaultAxesFontSize',14)
set(0,'DefaultLegendFontSize',14)

% Set LaTeX as default interpreter for axis labels, ticks and legends
set(0,'defaulttextinterpreter','latex')
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultLegendInterpreter','latex');

addpath('./Clothoids-develop/matlab');

% Set this flag to 0 to disable route planning and load a precomputed route
% to decrease simulation time
enable_routePlan = 0;

%% Open Simulink model
% Check if the Simulink model is already opened. Otherwise open it
openModels = find_system('SearchDepth', 0);
if (isempty(find(strcmp(openModels,'framework_sim'),1)))
    load_system('framework_sim');
    open_system('framework_sim');
end 

%% Load vehicle and auxiliary data
vehicle_data = getVehicleDataStruct();
tau_D = vehicle_data.steering_system.tau_D;  % [-] transmission ratio of steering mechanism

%% Define initial conditions for the simulation
X0 = loadInitialConditions;
initPose = loadInitialPose;

%% Simulation parameters
simulationPars = getSimulationParams(); 
Ts = simulationPars.times.step_size;  % integration step for the simulation (fixed step)
T0 = simulationPars.times.t0;         % starting time of the simulation
Tf = simulationPars.times.tf;         % stop time of the simulation

%% Load high-level planner parameters 
planner = loadHighLevelPlannerData();
HLP_sampleTime = planner.replan_time;

%% Load PID gain scheduling parameters
load PID_Gain_Scheduling.txt;
speed_vals_PID_sched = PID_Gain_Scheduling(:,1);
Kp_vals = PID_Gain_Scheduling(:,2);
Ki_vals = PID_Gain_Scheduling(:,3);
Kd_vals = PID_Gain_Scheduling(:,4);
N_vals  = PID_Gain_Scheduling(:,5);
Kb = 10;

%% Load lateral controller parameters
purePursuitParams   = purePursuitControllerParams();
stanleyParams       = stanleyControllerParams();
clothoidBasedParams = clothoidBasedControllerParams();

%% Select lateral controller type
select_lateralController;

%% Load road scenario
scenario = load('./Scenario/scenario');
costmap = scenario.scenario_data.costmap;
mapObjects = scenario.scenario_data.mapObjects;
vehicleDims = scenario.scenario_data.vehicleDims;

%% Define graphical interface settings
% Set this flag to 1 in order to enable online plots during the simulation
enable_onlinePlots = 0;
% Set this flag to 1 in order to enable a zoomed view in online simulation plots
enable_zoom = 0;

if (enable_onlinePlots)
    % Initialize figure for online plots
    figure('Name','Road Scenario','NumberTitle','off')
    grid on
    axis equal
    xlabel('x [m]')
    ylabel('y [m]')
    title('Road Scenario')
    hold on
end

%% Start Simulation
fprintf('Starting Simulation\n')
tic;
if (enable_routePlan)
    % Perform route planning
    routePlanner;
else
    % Load a precomputed route to decrease simulation time
    referencePath_points_original = load('refRoute_poses_RRT'); % RRT* solution
    referencePath_fewPoints = load('refPath_poses_fewPoints');  % interpolated RRT* solution with few points
    cpuTime_routePlan = load('cpuTime_routePlan');         
    refRoute_points_orig = referencePath_points_original.refRoute_points_orig;
    refRoute_fewPoints   = referencePath_fewPoints.refRoute_fewPoints;
    elapsed_time_routePlan = cpuTime_routePlan.elapsed_time_routePlan;    
end
% Simulink simulation
model_sim = sim('framework_sim.slx');
elapsed_time_simulation = toc; 
fprintf('Simulation completed\n')
total_simul_time = elapsed_time_simulation + elapsed_time_routePlan;
fprintf('It took %.1f seconds to compute the route with RRT*\n',elapsed_time_routePlan)
fprintf('The total simulation time is %.1f seconds\n',total_simul_time)

%% Post-Processing
dataAnalysis;
