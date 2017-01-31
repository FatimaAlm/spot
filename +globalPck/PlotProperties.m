classdef (Abstract) PlotProperties < handle
    % PlotProperties - abstract class that globally defines properties for
    % plotting class objects
    %
    % Syntax:
    %   no object constructor
    %
    % Other m-files required: none
    % Subfunctions: none
    % MAT-files required: none
    
    % Author:       Markus Koschi
    % Written:      15-November-2016
    % Last update:  31-January-2017
    %
    % Last revision:---
    
    %------------- BEGIN CODE --------------
    
    properties (Constant, GetAccess = public)
        SHOW_GRID = true;
        SHOW_AXIS = true;
        SHOW_OBJECT_NAMES = true;
        
        SHOW_LANES = true;
        SHOW_LANE_NAMES = false;
        SHOW_CENTER_VERTICES = false;
        
        SHOW_INITAL_CONFIGURATION = true;
        SHOW_OBSTACLES = true;
        COLOR_OBSTACLES = ['c', 'b', 'r', 'g', 'y', 'm'];
        SHOW_EGO_VEHICLE = true;
        SHOW_TRAJECTORIES = true;
        SHOW_OBSTACLES_CENTER = false;
        
        % predicted occupancy
        SHOW_PREDICTED_OCCUPANCIES = true;
        FACE_TRANSPARENCY = 0.2;      

        PLOT_OCC_SNAPSHOTS = false;
        PRINT_FIGURE = false;
    end
end

%------------- END CODE --------------