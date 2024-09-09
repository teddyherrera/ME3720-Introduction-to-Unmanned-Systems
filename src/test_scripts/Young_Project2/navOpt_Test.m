% ME3720 Lab1 PID Control w/ Fusion in UUV Simulator - Gazebo
% MATLAB and Gazebo
%           Gazebo frame (NED)         Matlab Frame
%          x(N)                            (y)
%            ^                              ^
%            |  /                           |  /
%            |~/  theta                     | /
%            |/                             |/) theta
%            ---------> y(E)                ---------> (x)
%

close all; clear; clc;

%% Waypoint navigation parameters
% define navigation waypoints/goals (1-x) from start(0)
% Note: These are in the Gazebo frame

waypoint0_x = -8; waypoint0_y = -8; waypoint0_z = -10; waypoint0_h = -90;
waypoint1_x = 6; waypoint1_y = 0; waypoint1_z = -10; waypoint1_h = -90;
waypoint2_x = 0; waypoint2_y = -6; waypoint2_z = -10; waypoint2_h = 0;
waypoint3_x = 0; waypoint3_y = 6; waypoint3_z = -10; waypoint3_h = -90;
waypoint4_x = 6; waypoint4_y = 0; waypoint4_z = -10; waypoint4_h = 180;

% create array of waypoints
WAYPOINT_x = [waypoint0_x, waypoint1_x, waypoint2_x, waypoint3_x, waypoint4_x];
WAYPOINT_y = [waypoint0_y, waypoint1_y, waypoint2_y, waypoint3_y, waypoint4_y];
WAYPOINT_z = [waypoint0_z, waypoint1_z, waypoint2_z, waypoint3_z, waypoint4_z];
WAYPOINT_h = [waypoint0_h, waypoint1_h, waypoint2_h, waypoint3_h, waypoint4_h];
%% Define world for creating occupancy map
worldSize = 24;     % meters
min  = -worldSize/2;
max  = worldSize/2;
resolution = 1;      % cells per meter
inflateVal = 1;
%% Define minimum and maximum values for position plot axes
% x/y are swapped due to Gazebo using NED reference frame
xplot_min = min-5;
xplot_max = max+5;
yplot_min = min-5;
yplot_max = max+5;

%% Create 2D occupancy map
ss = stateSpaceSE2;
sv = validatorOccupancyMap(ss);
% create matrix of zeros to feed into uccupancyMap function
mapMat = zeros(worldSize*resolution);
% Create occupancy map
map = occupancyMap(mapMat,resolution);
% Move location of map grid (bottom left corner) to desired location
map.GridLocationInWorld = [min,min];
% Fill in obstacles
for i = -4:(1/resolution):4
    for j = -4:(1/resolution):4
        setOccupancy(map,[i,j],1);  % Defines area of BOP
    end
end
map.inflate(inflateVal);
for i = min:(1/resolution):max      % Defines edges
    setOccupancy(map,[i,min],1);
    setOccupancy(map,[i,max],1);
    setOccupancy(map,[min,i],1);
    setOccupancy(map,[max,i],1);
end

sv.Map = map;
sv.ValidationDistance = 0.1;
ss.StateBounds = [map.XWorldLimits; map.YWorldLimits; [-pi pi]];
planner = plannerRRTStar(ss,sv, ...
          ContinueAfterGoalReached=true, ...
          MaxIterations=500, ...
          MaxConnectionDistance=2);
show(map)
% g = 1;
for i = 1:length(WAYPOINT_x)-1 %%%%% waypoint iteration
    % define start and end points in the Matlab reference frame
        % start will be read as the current vehicle position in simulation
    start = [WAYPOINT_y(i) WAYPOINT_x(i), WAYPOINT_h(i)];
        % goal will be the desired pose specified by the AprilTags
    goal = [WAYPOINT_y(i+1) WAYPOINT_x(i+1), WAYPOINT_h(i+1)];

    % Create array of values for the optimized navigation waypoints
    navPoints = [start];    % [ x(N) y(E) heading ]

    goalMet = false;

    rng(100,'twister') % repeatable result
    [pthObj,solnInfo] = plan(planner,start,goal);

    subplot((length(WAYPOINT_x)-1)/2,(length(WAYPOINT_x)-1)/2,i)
    % view([90 -90]);
    % set(gca,'XDir','reverse');
    map.show
    hold on
    % Tree expansion
    plot(solnInfo.TreeData(:,1),solnInfo.TreeData(:,2),'.-')
    % Draw path
    plot(pthObj.States(:,1),pthObj.States(:,2),'r-','LineWidth',2)

    % Find longest, valid, point-to-point distance along determined path
    % states_x = pthObj.States(:,1);
    % states_y = pthObj.States(:,2);
    % states_h = pthObj.States(:,3);
    count = 0; % iteration counter
    sStart = 1;
    while(goalMet == false) && (count < 10) %%% Optimization while loop
           
        for s = 1:length(pthObj.States)
            % pause(1)
            [isPathValid, lastValid] ...
                = isMotionValid(sv,pthObj.States(sStart,:),pthObj.States(s,:));
            if isPathValid
                endStep = lastValid;
                sLast = s;
            end
        end
        sStart = sLast;
        navPoints = [navPoints; endStep];
        if endStep == goal
            goalMet = true;
        else

        end
        count = count + 1;
        
    end %%% end optimization while loop

    % Determine length of paths
    lengthRRT = 0;
    lengthOpt = 0;
    for j = 1:length(pthObj.States)-1
        xj = pthObj.States(j+1,1) - pthObj.States(j,1);
        yj = pthObj.States(j+1,2) - pthObj.States(j,2);
        stepLength = sqrt(xj^2 + yj^2);
        lengthRRT = lengthRRT + stepLength;
    end
    for o =  1:length(navPoints)-1
        xo = navPoints(o+1,1) - navPoints(o,1);
        yo = navPoints(o+1,2) - navPoints(o,2);
        stepLength = sqrt(xo^2 + yo^2);
        lengthOpt = lengthOpt + stepLength;
    end
    
    legendRRT = sprintf('Planned Path: %4.2f m',lengthRRT);
    legendOpt = sprintf('Optimized Path: %4.2f m',lengthOpt);

    plot(start(1),start(2),'g*',MarkerSize=5,LineWidth=7)
    plot(goal(1),goal(2),'ro',MarkerSize=5,LineWidth=3)
    line(navPoints(:,1),navPoints(:,2),'Color','k','LineWidth',2)
    legend('',legendRRT,'Start','Goal',legendOpt, 'fontsize',12)
end %%%%% end waypoint iteration
  
