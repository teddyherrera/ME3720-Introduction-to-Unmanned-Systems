% ME3720 Lab1 PID Control w/ Fusion in UUV Simulator - Gazebo
% MATLAB and Gazebo

% Commands needed before running script
% ipaddress = 'http://172.20.137.195:11311'
% rosinit(ipaddress)
% rostopic list

%% Initialize Globals
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
desiredDepth = -45;
isFinalRun = true;
desiredRate = 2; %  hertz- times a second 
rate = rosrate(desiredRate);
dt = 1 / desiredRate; % Calculate dt based on the rosrate
kp_depth = 250.0; ki_depth = 12.5; kd_depth =  25.0; % Depth
kp_heading = 1.5; ki_heading = 0.5; kd_heading = 0.75; % Heading
stabilityThrust = 500; 
watchCircleRadius = 1.0;
numberToNavigate  = 3; % number of waypoints to navigate in random runs
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Occupancy map parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Adjustable parameters
worldSize = 24;     % meters
minX  = -worldSize/2;
maxX  = worldSize/2;
minY  = minX;
maxY  = maxX;
resolution = 1;      % cells per meter
inflateVal = 1;      % obstacle boundary inflation (meters)
valDist = 0.1;       % planner state variable validation distance
maxIterations = 500; % max number of iterations
maxDist = 2;         % maximum connection distance in meters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Navigation parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
rho = 2;                  % CTE "look ahead" distance
maxDepthError   = 1.5;    % max depth error for heading navigation
maxHeadingError = 10;     % max heading error to allow forward motion
maxPropulsion   = 20;     % max thruster RPM used for forward motion
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initialize PID controllers
depthPID = PIDcontroller(kp_depth, ki_depth, kd_depth);
headingPID = PIDcontroller(kp_heading, ki_heading, kd_heading);

%% Define waypoint objects using wpt = PoseDefinition(x,y,z,x_o,y_o,z_o,w)
wptStart = PoseDefinition(-3, -3, -48, 0, 0, 0, 1); % vehicle always starts here, 
wptInit  = PoseDefinition(-6, -6, -45, 0, 0, 0, 1); % always goes here first
wptHome1 = PoseDefinition(0, 0, -48.5, 0, 0, 0, 1); % ends here
wptHome2 = PoseDefinition(0, 0, -50, 0, 0, 0, 1);   % then lowers to here
%% AprilTag poses (exact)
% tag_00   = PoseDefinition(0.19, 5.657, -51.88, -0.005, 0.004, 0.745, -0.667);
% tag_01   = PoseDefinition(-0.023, 4.584, -55.786, -0.005, 0.001, 0.770, -0.638);
% tag_02   = PoseDefinition(-0.113, 4.397, -56.969, -0.006, 0.001, 0.745, -0.667);
% tag_03   = PoseDefinition(-0.120, -4.157, -51.989, -0.004, 0.0, -0.669, -0.743);
% tag_04   = PoseDefinition(-0.134, -4.028, -55.747, 0.0, 0.0, -0.680, -0.733);
% tag_05   = PoseDefinition(-0.17, -4.101, -57.023, 0.0, 0.0, -0.679, -0.734);
% tag_06   = PoseDefinition(3.980, -0.468, -51.756, 0.0, 0.0, -1.0, -0.016);
% tag_07   = PoseDefinition(3.935, -0.451, -52.127, 0.0, 0.0, -1.0, -0.16);
% tag_08   = PoseDefinition(3.942, -0.444, -52.606, 0.0, 0.0, -1.0, -0.016);
% tag_09   = PoseDefinition(3.946, -0.441, -52.920, 0.0, 0.0, -1.0, -0.016);
% tag_10   = PoseDefinition(3.950, -0.439, -53.250, 0.0, 0.0, -1.0, -0.017);
% tag_11   = PoseDefinition(3.383, 0.029, -54.338, 0.0, 0.0, -1.0, -0.013);

%% AprilTag Poses (modified)
tag_00   = PoseDefinition(0.0, 5.5, -51.88, -0.005, 0.004, 0.745, -0.667);
tag_01   = PoseDefinition(0.0, 5.5, -55.786, -0.005, 0.001, 0.770, -0.638);
tag_02   = PoseDefinition(0.0, 5.5, -56.969, -0.006, 0.001, 0.745, -0.667);
tag_03   = PoseDefinition(0.0, -5.5, -51.989, -0.004, 0.0, -0.669, -0.743);
tag_04   = PoseDefinition(0.0, -5.5, -55.747, 0.0, 0.0, -0.680, -0.733);
tag_05   = PoseDefinition(0.0, -5.5, -57.023, 0.0, 0.0, -0.679, -0.734);
tag_06   = PoseDefinition(5.5, 0.0, -51.756, 0.0, 0.0, -1.0, -0.016);
tag_07   = PoseDefinition(5.5, 0.0, -52.127, 0.0, 0.0, -1.0, -0.16);
tag_08   = PoseDefinition(5.5, 0.0, -52.606, 0.0, 0.0, -1.0, -0.016);
tag_09   = PoseDefinition(5.5, 0.0, -52.920, 0.0, 0.0, -1.0, -0.016);
tag_10   = PoseDefinition(5.5, 0.0, -53.250, 0.0, 0.0, -1.0, -0.017);
tag_11   = PoseDefinition(5.5, 0.0, -54.338, 0.0, 0.0, -1.0, -0.013);
% tags = [tag_00, tag_01, tag_02, tag_03, tag_04, tag_05, tag_06, ...
%         tag_07, tag_08, tag_09, tag_10, tag_11];

% tag_00   = PoseDefinition(5.5, 0, -51.88, -0.005, 0.004, 0.745, -0.667);
% tag_01   = PoseDefinition(5.5, 0, -55.786, -0.005, 0.001, 0.770, -0.638);
% tag_02   = PoseDefinition(5.5, 0, -56.969, -0.006, 0.001, 0.745, -0.667);
% tag_03   = PoseDefinition(-5.5, 0, -51.989, -0.004, 0.0, -0.669, -0.743);
% tag_04   = PoseDefinition(-5.5, 0, -55.747, 0.0, 0.0, -0.680, -0.733);
% tag_05   = PoseDefinition(-5.5, 0, -57.023, 0.0, 0.0, -0.679, -0.734);
% tag_06   = PoseDefinition(0, 5.5, -51.756, 0.0, 0.0, -1.0, -0.016);
% tag_07   = PoseDefinition(0, 5.5, -52.127, 0.0, 0.0, -1.0, -0.16);
% tag_08   = PoseDefinition(0, 5.5, -52.606, 0.0, 0.0, -1.0, -0.016);
% tag_09   = PoseDefinition(0, 5.5, -52.920, 0.0, 0.0, -1.0, -0.016);
% tag_10   = PoseDefinition(0, 5.5, -53.250, 0.0, 0.0, -1.0, -0.017);
% tag_11   = PoseDefinition(0, 5.5, -54.338, 0.0, 0.0, -1.0, -0.013);
tags = [tag_00, tag_01, tag_02, tag_03, tag_04, tag_05, tag_06, ...
        tag_07, tag_08, tag_09, tag_10, tag_11];

% Waypoint array for final run

% wpt sequence for final evaluated run
finalRun = [wptInit, tag_00, tag_05, tag_10, tag_01, wptHome1, wptHome2];
% wpt sequence for non-final runs where wpts between start and end are
% random
% otherRun = [wptInit, wptHome1, wptHome2];
mid1 = tag_05; % only used as place holder
mid2 = tag_10; % only used as place holder
otherRun = [wptInit, tag_00, mid1, mid2, wptHome1, wptHome2];

% Define minimum and maximum values for position plot axes
% x/y are swapped due to Gazebo using NED reference frame
% xplot_min = min(WAYPOINT_y) - 5;
% xplot_max = max(WAYPOINT_y) + 5;
% yplot_min = min(WAYPOINT_x) - 5;
% yplot_max = max(WAYPOINT_x) + 5;
xplot_min = -15;
xplot_max = 15;
yplot_min = -15;
yplot_max = 15;

%% create the publish object
bow_port_thruster_pub = rospublisher('/bow_port_thruster', 'std_msgs/Float64');
bow_stbd_thruster_pub = rospublisher('/bow_stbd_thruster', 'std_msgs/Float64');
vert_port_thruster_pub = rospublisher('/vert_port_thruster', 'std_msgs/Float64');
vert_stbd_thruster_pub = rospublisher('/vert_stbd_thruster', 'std_msgs/Float64');
aft_port_thruster_pub = rospublisher('/aft_port_thruster', 'std_msgs/Float64');
aft_stbd_thruster_pub = rospublisher('/aft_stbd_thruster','std_msgs/Float64');
aft_vert_thruster_pub = rospublisher('/aft_vert_thruster', 'std_msgs/Float64');

%% create the publish message
bow_port_thruster_msg = rosmessage(bow_port_thruster_pub);
bow_stbd_thruster_msg = rosmessage(bow_stbd_thruster_pub);
vert_port_thruster_msg = rosmessage(vert_port_thruster_pub);
vert_stbd_thruster_msg = rosmessage(vert_stbd_thruster_pub);
aft_port_thruster_msg = rosmessage(aft_port_thruster_pub);
aft_stbd_thruster_msg = rosmessage(aft_stbd_thruster_pub);
aft_vert_thruster_msg = rosmessage(aft_vert_thruster_pub);

%% subscribe to gazebo odometry and April Tag messages
fusion_state_sub = rossubscriber('/fusion/pose_gt');
april_tag_sub    = rossubscriber('/apriltag_wypt_msg');

%% Initialize Depth Plotting
figure(1);
subplot(2, 1, 1);
depthPlot = plot(0, desiredDepth, 'b-', 'LineWidth', 2);
hold on;
targetDepthPlot = plot(0, 0, 'r--', 'LineWidth', 2);
xlabel('Samples');
ylabel('Depth (m)');
title('Depth Tracking');
legend('Actual Depth', 'Desired Depth', 'Location','bestoutside');

subplot(2, 1, 2);
depthErrorPlot = plot(0, 0, 'k-', 'LineWidth', 2);
hold on
depthReferenceError = plot(0,'r--', 'LineWidth', 2);
xlabel('Samples');
ylabel('Error (m)');
title('Depth Error');
legend('est. error', 'desired error', 'Location','bestoutside');

sgtitle(['Depth PID Parameters:',...
         ' ROSrate = ',num2str(desiredRate),' Hz ',...
         ' Kp = ',num2str(kp_depth),...
         ' Ki = ',num2str(ki_depth),...
         ' Kd = ',num2str(kd_depth)]);
hold off

%% Initialize Heading Plot
figure(2);
subplot(2, 1, 1);
headingPlot = plot(0, 0, 'b-', 'LineWidth', 2);
hold on;
targetHeadingPlot = plot(0, 0, 'r--', 'LineWidth', 2);
xlabel('Heading (Degrees)');
ylabel('Samples');
title('Heading Tracking');
legend('Actual Heading', 'Desired Heading', 'Location','bestoutside');

subplot(2, 1, 2);
headingErrorPlot = plot(0, 0, 'k-', 'LineWidth', 2);
hold on
headingReferenceError = plot(0,'r--', 'LineWidth', 2);
xlabel('Error (degrees)');
ylabel('Samples');
title('Heading Error');
legend('est. error', 'desired error', 'Location','bestoutside');
sgtitle(['Heading PID Parameters: ',...
         'ROSrate = ',num2str(desiredRate),' Hz ',...
         ' Kp = ',num2str(kp_heading),...
         ' Ki = ',num2str(ki_heading),...
         ' Kd = ',num2str(kd_heading)])
hold off

%% Plot navigation box and vehicle position
%{
figure(3);
hold on;
axis([yplot_min yplot_max xplot_min xplot_max]);

set(gca,'XDir','reverse');
plot(WAYPOINT_y, WAYPOINT_x, 'ro', MarkerSize=5);
line(WAYPOINT_y, WAYPOINT_x);
fusPos = plot(0,0,'b.', MarkerSize=6); % this can be updated to initialize with initial position
title('Waypoint Navigation');
xlabel('Y position (East)');
ylabel('X position (North)');
%}
%% Check to see if odometry has been recieved 
x =[];
not_received_odometry = 1;
k = 0;

while(not_received_odometry)
    k = k+1;
    statedata = receive(fusion_state_sub,3);
    posit = statedata.Pose.Pose.Position;
    x = posit.X;
    y = posit.Y;
    z = posit.Z;

    if (~isempty(x))
        not_received_odometry = 0;
    end
    waitfor(rate);
    disp('waiting for odometry');
end
disp('received odometry');

%% Variables for updating plots
currentYdata= [];
currentXdata = [];
plotHeadingSample = [];
plotDepthSample = [];
actualDepthData = [];
actualHeadingData = [];
depthErrorData = [];
headingErrorData = [];
desiredHeadingData = [];

%% Main Control Loop %%%%%%%%%%%%%%%%%%

%% Create 2D occupancy map

ss = stateSpaceSE2;
mapMat = zeros(worldSize*resolution);
% Map with obstacles
[obsMap, ssObs,  svObs ] ...
    = findNavigationMap(ss, mapMat, minX, minY, resolution, valDist);
% open map for navigation above level of BOP
[openMap,ssOpen, svOpen] ...
    = findNavigationMap(ss, mapMat, minX, minY, resolution, valDist);
% Fill in obstacles
[obsMap] = fillOccupancy(obsMap, resolution, inflateVal, minX, maxX);

plannerObs = plannerRRTStar(ss,svObs);
          plannerObs.ContinueAfterGoalReached=true;
          plannerObs.MaxIterations=maxIterations;
          plannerObs.MaxConnectionDistance=maxDist;

plannerOpen = plannerRRTStar(ss,svOpen);
          plannerOpen.ContinueAfterGoalReached=true;
          plannerOpen.MaxIterations=maxIterations;
          plannerOpen.MaxConnectionDistance=maxDist;

%%

tic;
if isFinalRun
    navArray = finalRun;
else
    navArray = otherRun;
end
ind = 1;

isLastPoint = false;
start = [wptStart.x, wptStart.y, wptStart.h];       %%%%%%%%%%%%%%%%%%%%%%%
while (ind <= length(navArray)) % global navigation loop

    if ind == 1 || ind > (length(navArray)-2)
        map = svOpen.Map;
        sv = svOpen;
        ss = ssOpen;
        planner = plannerOpen;
    else
        map = svObs.Map;
        sv = svObs;
        ss = ssObs;
        planner = plannerObs;
    end
  
    % AprilTag data
    nextGoal  = [];
    isTagRead = false;  % change to true once AprilTag msg is received

    desiredDepth = navArray(ind).z;
    goal = [navArray(ind).x, navArray(ind).y, navArray(ind).h]; %%%%%%%%%%%%%%%%%%%%%
    navPoints = findNavigationPath(sv, start, goal, planner, ind);

    
    %% Navigation from one point to next
    jj = 1;
    while (jj < length(navPoints)) % single path navigation loop
        %% recieve updated pose
        statedata = receive(fusion_state_sub, 3);
        posit = statedata.Pose.Pose.Position;
        currentX = posit.X;  %% Y in Matlab  
        currentY = posit.Y;  %% X in Matlab
        currentDepth = posit.Z;
        quat = statedata.Pose.Pose.Orientation;
        angles = quat2eul([quat.W quat.X quat.Y quat.Z]); %%%% May need to swap Y/X % Default format ZYX 
        currentHeading = wrapTo180(rad2deg(angles(1)));
    
        %% Cross Track Error
        desiredHeading = CTE(currentX, currentY, ...
                 navPoints(jj,1), navPoints(jj,2), ...
                 navPoints(jj+1,1), navPoints(jj+1,2), ...
                 rho);
    
        %% Implement controllers and Conditional Logic
        % PID control for depth
        [depthControlSignal, depthError] ...
                    = depthPID.update(currentDepth, desiredDepth, dt);
    
        if abs(depthError) >= maxDepthError
            %% Update message data with control signal
            [vert_port_thruster_msg, vert_stbd_thruster_msg, ...
                bow_port_thruster_msg, bow_stbd_thruster_msg, ...
                aft_port_thruster_msg, aft_stbd_thruster_msg] = ...
                setDepth(depthControlSignal, stabilityThrust, ...
                    vert_port_thruster_msg, vert_stbd_thruster_msg, ...
                    bow_port_thruster_msg, bow_stbd_thruster_msg, ...
                    aft_port_thruster_msg, aft_stbd_thruster_msg, ...
                    aft_vert_thruster_msg);
            %% Send message to Fusion
            send(vert_stbd_thruster_pub, vert_stbd_thruster_msg);
            send(vert_port_thruster_pub, vert_port_thruster_msg);
            send(aft_port_thruster_pub, aft_port_thruster_msg);
            send(aft_stbd_thruster_pub, aft_stbd_thruster_msg);
            send(bow_port_thruster_pub, bow_port_thruster_msg);
            send(bow_stbd_thruster_pub, bow_stbd_thruster_msg);
            % send(aft_vert_thruster_pub, aft_vert_thruster_msg);
           
        else
            sample = toc;
            plotHeadingSample = [plotHeadingSample, sample];
    
            % PID control for heading
            [headingControlSignal, headingError] ...
                = headingPID.updateAngle(currentHeading, desiredHeading, dt);
            if abs(headingError) >= maxHeadingError 
                rpmAdjustment = 0;
                %% Update Message Data
                [vert_port_thruster_msg, vert_stbd_thruster_msg, ...
                    bow_port_thruster_msg, bow_stbd_thruster_msg, ...
                    aft_port_thruster_msg, aft_stbd_thruster_msg] = ...
                    updateHeading(depthControlSignal, stabilityThrust,...
                        headingControlSignal, rpmAdjustment, ... 
                        vert_port_thruster_msg, vert_stbd_thruster_msg, ...
                        bow_port_thruster_msg, bow_stbd_thruster_msg, ...
                        aft_port_thruster_msg, aft_stbd_thruster_msg, ...
                        aft_vert_thruster_msg);
                %% Send message to Fusion
                % send(vert_stbd_thruster_pub, vert_stbd_thruster_msg);
                % send(vert_port_thruster_pub, vert_port_thruster_msg);
                % send(aft_port_thruster_pub, aft_port_thruster_msg);
                % send(aft_stbd_thruster_pub, aft_stbd_thruster_msg);
                % send(bow_port_thruster_pub, bow_port_thruster_msg);
                % send(bow_stbd_thruster_pub, bow_stbd_thruster_msg);
                % % send(aft_vert_thruster_pub, aft_vert_thruster_msg);
    
            else
                rpmAdjustment = maxPropulsion;
                %% Update Message Data
                [vert_port_thruster_msg, vert_stbd_thruster_msg, ...
                    bow_port_thruster_msg, bow_stbd_thruster_msg, ...
                    aft_port_thruster_msg, aft_stbd_thruster_msg] = ...
                    updateHeading(depthControlSignal, stabilityThrust, ...
                        headingControlSignal, rpmAdjustment, ... 
                        vert_port_thruster_msg, vert_stbd_thruster_msg, ...
                        bow_port_thruster_msg, bow_stbd_thruster_msg, ...
                        aft_port_thruster_msg, aft_stbd_thruster_msg, ...
                        aft_vert_thruster_msg);
                %% Send message to Fusion
                % send(vert_stbd_thruster_pub, vert_stbd_thruster_msg);
                % send(vert_port_thruster_pub, vert_port_thruster_msg);
                % send(aft_port_thruster_pub, aft_port_thruster_msg);
                % send(aft_stbd_thruster_pub, aft_stbd_thruster_msg);
                % send(bow_port_thruster_pub, bow_port_thruster_msg);
                % send(bow_stbd_thruster_pub, bow_stbd_thruster_msg);
                % % send(aft_vert_thruster_pub, aft_vert_thruster_msg);
    
            end
            %% Send message to Fusion
            send(vert_stbd_thruster_pub, vert_stbd_thruster_msg);
            send(vert_port_thruster_pub, vert_port_thruster_msg);
            send(aft_port_thruster_pub, aft_port_thruster_msg);
            send(aft_stbd_thruster_pub, aft_stbd_thruster_msg);
            send(bow_port_thruster_pub, bow_port_thruster_msg);
            send(bow_stbd_thruster_pub, bow_stbd_thruster_msg);
            % send(aft_vert_thruster_pub, aft_vert_thruster_msg);
    
            %% Collect data for plotting: Heading
            actualHeadingData = [actualHeadingData, currentHeading];
            desiredHeadingData = [desiredHeadingData, desiredHeading];
            headingErrorData = [headingErrorData, headingError];
    
            %% Update Heading Plots
            set(headingPlot, 'XData', actualHeadingData, 'YData', plotHeadingSample);
            set(targetHeadingPlot,'Xdata',  desiredHeadingData,'YData',plotHeadingSample);
            set(headingErrorPlot,'XData',  headingErrorData,'YData', plotHeadingSample);
            set(headingReferenceError,'XData', repmat(0, length(plotHeadingSample),1),'YData', plotHeadingSample);
        end
    
        %% Collect data for plotting: Depth
        plotDepthSample = [plotDepthSample, length(actualDepthData)+1];
        actualDepthData = [actualDepthData, currentDepth];
        depthErrorData = [depthErrorData, depthError];
    
        %% Update Depth plots
        set(depthPlot, 'XData', plotDepthSample, 'YData', actualDepthData);
        set(targetDepthPlot, 'XData', plotDepthSample, 'YData', repmat(desiredDepth, length(actualDepthData), 1));
        set(depthErrorPlot, 'XData', plotDepthSample, 'YData', depthErrorData);
        set(depthReferenceError, 'XData', plotDepthSample, 'YData', repmat(0, length(plotDepthSample),1));
        drawnow;
    
        %% Update Waypoint Navigation plot
        % currentYdata = [currentYdata, currentY];
        % currentXdata = [currentXdata, currentX];
        % figure(ind);
        % hold on;
        % fusPos = plot(0,0,'b.', MarkerSize=6);
        % set(fusPos, 'Xdata', currentYdata, 'YData', currentXdata);
        % drawnow;
        
        %% Evaluate if within parameter to say objective is met
        current_distance_to_next_waypoint ...
            = norm([navPoints(jj+1,1) - currentX,...
                    navPoints(jj+1,2) - currentY]);
        proximityThreshold = watchCircleRadius;
        if (current_distance_to_next_waypoint < proximityThreshold)
            jj = jj + 1;
        end
    
        waitfor(rate);
        iterationTime = toc;
    end % end single path navigation loop

    % read tags before moving to next waypoint, except for after the first
    % waypoint (navigating off the BOP) and the penultimate and final
    % waypoints (navigating to "home" position)
    if ind > 1 && ind < (length(navArray)-2) 
        desiredHeading = goal(3);
        % turn off propulsion thrusters
        rpmAdjustment = 0;
        % turn to desired heading
        % [headingControlSignal, headingError] ...
        %   = headingPID.updateAngle(currentHeading, desiredHeading, dt);
    
        % read AprilTag
        while isTagRead == false % read tag while loop
            [headingControlSignal, headingError] ...
                = headingPID.updateAngle(currentHeading, desiredHeading, dt);
    
            % Update Message Data
            [vert_port_thruster_msg, vert_stbd_thruster_msg, ...
                bow_port_thruster_msg, bow_stbd_thruster_msg, ...
                aft_port_thruster_msg, aft_stbd_thruster_msg] = ...
                updateHeading(depthControlSignal, stabilityThrust, ...
                    headingControlSignal, rpmAdjustment, ... 
                    vert_port_thruster_msg, vert_stbd_thruster_msg, ...
                    bow_port_thruster_msg, bow_stbd_thruster_msg, ...
                    aft_port_thruster_msg, aft_stbd_thruster_msg, ...
                    aft_vert_thruster_msg);
            % Send message to Fusion
            send(vert_stbd_thruster_pub, vert_stbd_thruster_msg);
            send(vert_port_thruster_pub, vert_port_thruster_msg);
            send(aft_port_thruster_pub, aft_port_thruster_msg);
            send(aft_stbd_thruster_pub, aft_stbd_thruster_msg);
            send(bow_port_thruster_pub, bow_port_thruster_msg);
            send(bow_stbd_thruster_pub, bow_stbd_thruster_msg);
            % send(aft_vert_thruster_pub, aft_vert_thruster_msg);
    
            %check for AprilTag msg
            tagData   = receive(april_tag_sub);
            %%%%% Using AprilTag pose data %%%%%
            % nextGoal  = tagData.Pose.Pose.Position; 
            % if isempty(nextGoal) == false
            %     isTagRead = true;
            %     nextX     = nextGoal.X;
            %     nextY     = nextGoal.Y;
            %     nextZ     = nextGoal.Z;
            %     quat      = statedata.Pose.Pose.Orientation;
            %     angles    = quat2eul([quat.W quat.X quat.Y quat.Z]);
            %     nextHeading = wrapTo180(rad2deg(angles(1)));
            % end

            %%%%% Using AprilTag ID number %%%%%
            nextGoal  = str2num(tagData.ChildFrameId)
            if isempty(nextGoal) == false
                % Check if new goal (x,y) same as current goal (x,y)
                % if 
                
                isTagRead = true;
                if isFinalRun == false
                    nextTag = tags(nextGoal + 1);
                    % nextX     = nextTag.x;
                    % nextY     = nextTag.y;
                    % nextZ     = nextTag.z;
                    % nextHeading = nextTag.h;
                    navArray(ind+1) ...
                        = PoseDefinition(nextTag.x, nextTag.y, ...
                                         nextTag.z, nextTag.h);
                end
                break
            end
    
        end % end read tag while loop
    % else
    %     nextX = navArray(ind+1).x;
    %     nextY = navArray(ind+1).y;
    %     nextZ = navArray(ind+1).z;
    %     nextHeading = navArray(ind+1).h;
    end % end if ind>1 loop  


    start = goal;
    ind = ind + 1;

    % if (isFinalRun == true)
    % 
    % else
    % end

    % if (isFinalRun == false)
    %     if ind < numberToNavigateavigate + 2
    %         % navArray = [navArray, wptHome1, wptHome2];
    %         isFinalRun = true;
    %     else
    %         % % get pose from next AprilTag and add to navArray
    %         nextStep = PoseDefinition(nextX,nextY,nextZ, nextHeading);
    %         navArray = [navArray,nextStep];
    % 
    %         % Add evaluation for isLastPoint
    %             % Still need this criteria from professor
    % 
    % 
    %     end
    % end

end % end global navigation loop


% Things to implement if time permits
%       - Use faster propulsion, slow down as approach wpt
%       - hover capability
%       - checks/corrections for overshooting target
%       - 
%
%
%
%
%
%



