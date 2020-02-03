clc;
clearvars;

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


% ----------------------------------------------------
%% Build costmap
% ----------------------------------------------------

mapWidth = 350;
mapLength = 350;
costVal = 0.1;
cellSize = 1;

% ---------------------------
% collision-checking properties
% ---------------------------
% load vehicle data
vehicle_data = getVehicleDataStruct();
vehWheelbase = vehicle_data.vehicle.L;
Wr = vehicle_data.vehicle.Wr;
Wf = vehicle_data.vehicle.Wf;
vehWidth = max(Wr,Wf); 
vehHeight = 1.4;
overhang_length = 1.5;
vehLength = vehWheelbase+overhang_length; 
vehicleDims = vehicleDimensions(vehLength,vehWidth,vehHeight, ...
    'FrontOverhang',2*overhang_length/3,'RearOverhang',1*overhang_length/3);
distFromSide = 0.175;
centerPlacements = [distFromSide 0.5 1-distFromSide];
inflationRadius = 1.2;
ccConfig = inflationCollisionChecker(vehicleDims, ...
    'CenterPlacements',centerPlacements,'InflationRadius',inflationRadius);

% ---------------------------
% plot the vehicle
% --------------------------- 
if (~enable_docked)
    figure('Name','Vehicle','NumberTitle','off','Position',[0,0,500,1000]), clf 
    set(gcf,'units','points','position',[150,150,600,350])
else
    figure('Name','Vehicle','NumberTitle','off'), clf
end  
plot(ccConfig)

% create a map with uniform costs and without obstacles
costmap = vehicleCostmap(mapWidth,mapLength,costVal,'CellSize',cellSize,...
                    'CollisionChecker',ccConfig);

occupiedVal = 0.9;  % used for all obstacles

% ---------------------------
% fixed obstacles positions
% ---------------------------
% first wall
[first_wall_Xcoord,first_wall_Ycoord] = meshgrid(0:300,100:103);
first_wall = [first_wall_Xcoord(:),first_wall_Ycoord(:)];
% second wall - part 1
[second_wall_1_Xcoord,second_wall_1_Ycoord] = meshgrid(0:50,200:203);
second_wall_1 = [second_wall_1_Xcoord(:),second_wall_1_Ycoord(:)];
% second wall - part 2
[second_wall_2_Xcoord,second_wall_2_Ycoord] = meshgrid(70:350,200:203);
second_wall_2 = [second_wall_2_Xcoord(:),second_wall_2_Ycoord(:)];
% obstacle between the two walls
[obst_betweenWalls_Xcoord,obst_betweenWalls_Ycoord] = meshgrid(130:160,130:180);
obst_betweenWalls = [obst_betweenWalls_Xcoord(:),obst_betweenWalls_Ycoord(:)];
% obstacle next to the parking lot
[obst_nextToParkingLot_Xcoord,obst_nextToParkingLot_Ycoord] = meshgrid(250:260,230:320);
obst_nextToParkingLot = [obst_nextToParkingLot_Xcoord(:),obst_nextToParkingLot_Ycoord(:)];
% parking lot
[parkingLot_rightBoarder_Xcoord,parkingLot_rightBoarder_Ycoord] = meshgrid(345:350,300:300);
[parkingLot_leftBoarder_Xcoord,parkingLot_leftBoarder_Ycoord] = meshgrid(345:350,290:290);
parkingLot_rightBoarder = [parkingLot_rightBoarder_Xcoord(:),parkingLot_rightBoarder_Ycoord(:)];
parkingLot_leftBoarder = [parkingLot_leftBoarder_Xcoord(:),parkingLot_leftBoarder_Ycoord(:)];
parkingLot = [parkingLot_rightBoarder; parkingLot_leftBoarder];

% full set of obstacles positions
obstacles_set = [first_wall; second_wall_1; second_wall_2; obst_betweenWalls; obst_nextToParkingLot; parkingLot_rightBoarder; parkingLot_leftBoarder];

setCosts(costmap,obstacles_set,occupiedVal)

% ---------------------------
% plot the scenario
% --------------------------- 
if (~enable_docked)
    figure('Name','Scenario','NumberTitle','off','Position',[0,0,500,1000]), clf 
    set(gcf,'units','points','position',[150,150,600,350])
else
    figure('Name','Scenario','NumberTitle','off'), clf
end   
plot(costmap)
grid on
axis equal
xlabel('x [m]')
ylabel('y [m]')


% ----------------------------------------------------
%% Save the scenario
% ----------------------------------------------------

scenario_data.costmap = costmap;
scenario_data.vehicleDims = vehicleDims;
% ---------------------------
% save the scenario objects as vertices of rectangles
% --------------------------- 
parkingLot_rightBoarder_rect = [min(parkingLot_rightBoarder(:,1)), min(parkingLot_rightBoarder(:,2)), ...
                                         max(parkingLot_rightBoarder(:,1))-min(parkingLot_rightBoarder(:,1)), max(parkingLot_rightBoarder(:,2))-min(parkingLot_rightBoarder(:,2))];
parkingLot_leftBoarder_rect = [min(parkingLot_leftBoarder(:,1)), min(parkingLot_leftBoarder(:,2)), ...
                                         max(parkingLot_leftBoarder(:,1))-min(parkingLot_leftBoarder(:,1)), max(parkingLot_leftBoarder(:,2))-min(parkingLot_leftBoarder(:,2))];                                     
first_wall_rect = [min(first_wall(:,1)), min(first_wall(:,2)), ...
                            max(first_wall(:,1))-min(first_wall(:,1)), max(first_wall(:,2))-min(first_wall(:,2))];
second_wall_1_rect = [min(second_wall_1(:,1)), min(second_wall_1(:,2)), ...
                               max(second_wall_1(:,1))-min(second_wall_1(:,1)), max(second_wall_1(:,2))-min(second_wall_1(:,2))];
second_wall_2_rect = [min(second_wall_2(:,1)), min(second_wall_2(:,2)), ...
                               max(second_wall_2(:,1))-min(second_wall_2(:,1)), max(second_wall_2(:,2))-min(second_wall_2(:,2))];
obst_betweenWalls_rect = [min(obst_betweenWalls(:,1)), min(obst_betweenWalls(:,2)), ...
                                   max(obst_betweenWalls(:,1))-min(obst_betweenWalls(:,1)), max(obst_betweenWalls(:,2))-min(obst_betweenWalls(:,2))];
obst_nextToParkingLot_rect = [min(obst_nextToParkingLot(:,1)), min(obst_nextToParkingLot(:,2)), ...
                                       max(obst_nextToParkingLot(:,1))-min(obst_nextToParkingLot(:,1)), max(obst_nextToParkingLot(:,2))-min(obst_nextToParkingLot(:,2))];
scenario_data.mapObjects = [parkingLot_rightBoarder_rect;
                            parkingLot_leftBoarder_rect;
                            first_wall_rect;
                            second_wall_1_rect;
                            second_wall_2_rect;
                            obst_betweenWalls_rect;
                            obst_nextToParkingLot_rect];                                  
save('./Scenario/scenario','scenario_data');


