% main - SPOT: Set-Based Prediction Of Traffic Participants
%
% Syntax:
%   main()
%
% Inputs:
%   none
% User input is defined within the code below:
%       inputFile - input file (in XML format)
%       time intervals - time stamps for the prediction and visualization
%
% Outputs:
%   perception - object that contains a map with obstacles and their
%                occupancies
%
% Other m-files required: none
% Subfunctions: none
% MAT-files required: none
%
% See also: Althoff and Magdici, 2016, Set-Based Prediction of Traffic
% Participants on Arbitrary Road Networks

% Author:       Markus Koschi
% Written:      06-September-2016
% Last update:  31-January-2017
%
% First version by: Matthias Althoff, Silvia Magdici
% Last revision:---

%------------- BEGIN CODE --------------


%--- Preliminaries ---
clc; clear;
close all;


%--- User Input ---
% define the input here: (inputFile = someFile.ext;)

inputFile = 'scenarios/Intersection_Leopold_Hohenzollern_1.osm';
%inputFile = 'scenarios/Intersection_Leopold_Hohenzollern_2.osm';
%inputFile = 'scenarios/Intersection_Leopold_Hohenzollern_3.osm';
%inputFile = 'scenarios/Krailling_2Lanes_OppositeDirection_Narrow_WithTraffic.osm';
%inputFile = 'scenarios/Fuerstenfeldbruck_T_junction_WithTraffic.osm';
%inputFile = 'scenarios/Garching_highway_4Lanes_SameDirection_With_SlipRoad_WithTraffic.osm';
%inputFile = 'scenarios/GarchingNorth_2Lanes_SameDirection_Merging_WithTraffic.osm';
%inputFile = 'scenarios/Munich_InTheCity_2Lanes_SameDirection_WithTraffic.osm';
%inputFile = 'scenarios/MunichFeldmoching_highway_3Lanes_SameDirection_WithTraffic.osm';
%inputFile = 'scenarios/BergkirchenB471_2Lanes_OppositeDirection_WithTraffic.osm';
%inputFile = 'scenarios/Fuerstenfeldbruck_Intersection_WithTraffic.osm';
%inputFile = 'scenarios/A9_Freimann_1_direction.osm';
%inputFile = 'scenarios/NGSIM_US101_0.osm';
%inputFile = 'scenarios/merging_1vehicle_egoVehicle.osm';
%inputFile = 'scenarios/merging_2vehicles_1.osm';
%inputFile = 'scenarios/merging_2vehicles_2.osm';
%inputFile = 'scenarios/Example_rightTurn.osm';
%inputFile = 'scenarios/Example_straight_2lanes.osm';


% time interval in seconds for prediction of the occupancy
ts = 0;
dt = 0.5;
tf = 2;

% time interval in seconds for visualization
ts_plot = 0;
dt_plot = 0.5;
tf_plot = 2;


%--- Main code ---
% create perception from input (holding a map with all lanes, adjacency
% graph and all obstacles)
perception = globalPck.Perception(inputFile, ts);

% create time interval for occupancy calculation
timeInterval = globalPck.TimeInterval(ts, dt, tf);

% plot initial configuration
if globalPck.PlotProperties.SHOW_INITAL_CONFIGURATION
    figure('Name','Initial configuration')
    perception.plot();
    if globalPck.PlotProperties.PRINT_FIGURE
        saveas(gcf,'Initial configuration','epsc')
    end
end

% do occupancy calculation
perception.computeOccupancyGlobal(timeInterval);


%--- Plot the perception (all lanes and all obstacles incl. occupancies) ---
% create time interval for plot
timeInterval_plot = globalPck.TimeInterval(ts_plot, dt_plot, tf_plot);

% do plot
figure('Name', 'Perception and Prediction')
perception.plot(timeInterval_plot)
if globalPck.PlotProperties.PRINT_FIGURE
    saveas(gcf,'Perception and Prediction','epsc')
end

%--- Plot the occupancy step by step (snapshots) ---
if globalPck.PlotProperties.PLOT_OCC_SNAPSHOTS
    i = 1;
    for t = ts_plot:dt_plot:(tf_plot-dt_plot)
        % create time interval for plot
        timeInterval_plot = globalPck.TimeInterval(ts_plot+t, dt_plot, ts_plot+t+dt_plot);
        
        % do plot
        name_plot = ['Occ_Snapshot_' num2str(i)];
        i = i + 1;
        figure('Name',name_plot)
        perception.plot(timeInterval_plot)
        
        if globalPck.PlotProperties.PRINT_FIGURE
            saveas(gcf,name_plot,'epsc')
        end
    end
end

%------------- END CODE --------------