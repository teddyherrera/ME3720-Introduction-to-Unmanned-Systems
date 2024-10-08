% ME3720 Lab1 PID Control w/ Fusion in UUV Simulator - Gazebo
% MATLAB and Gazebo

% Commands needed before running script
% ipaddress = 'http://172.20.137.195:11311'
% rosinit(ipaddress)
% rostopic list

%% Initialize Globals
desiredDepth = -10;
desiredRate = 2; %  hertz- times a second 
rate = rosrate(desiredRate);
dt = 1 / desiredRate; % Calculate dt based on the rosrate
kp_depth = 300.0; ki_depth = 10.0; kd_depth =  50.0; % Depth
kp_heading = 1; ki_heading = .2; kd_heading = .8; % Heading
stabilityThrust = 500; 

%% Initialize PID controllers
depthPID = PIDcontroller(kp_depth, ki_depth, kd_depth);
headingPID = PIDcontroller(kp_heading, ki_heading, kd_heading);

%% Waypoint navigation parameters
% define waypoints, vehicle will navigate from initial location to wpt1
% along the heading from wpt0 to wpt1. It will then continue on the lines
% from wpt1-wpt2, wpt2-wpt3, wpt3-wpt4, wpt4-wpt1
waypoint1_x = 25; waypoint1_y = 25; waypoint1_z = -10;
waypoint2_x = 50; waypoint2_y = 25; waypoint2_z = -10;
waypoint3_x = 50; waypoint3_y = 50; waypoint3_z = -10;
waypoint4_x = 25; waypoint4_y = 50; waypoint4_z = -10;

WAYPOINT_x = [waypoint1_x, waypoint2_x, waypoint3_x, waypoint4_x, waypoint1_x];
WAYPOINT_y = [waypoint1_y, waypoint2_y, waypoint3_y, waypoint4_y, waypoint1_y];
WAYPOINT_z = [waypoint1_z, waypoint2_z, waypoint3_z, waypoint4_y, waypoint1_z];

% Define minimum and maximum values for position plot axes
% x/y are swapped due to Gazebo using NED reference frame
xplot_min = min(WAYPOINT_y) - 25;
xplot_max = max(WAYPOINT_y) + 50;
yplot_min = min(WAYPOINT_x) - 25;
yplot_max = max(WAYPOINT_x) + 25;

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

%% subscribe to gazebo odometry
fusion_state_sub = rossubscriber('/fusion/pose_gt');

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
figure(3);
hold on;
axis([yplot_min yplot_max xplot_min xplot_max]);
plot(WAYPOINT_y, WAYPOINT_x, 'ro', MarkerSize=5);
line(WAYPOINT_y, WAYPOINT_x);
fusPos = plot(0,0,'b.', MarkerSize=6); % this can be updated to initialize with initial position
title('Waypoint Navigation');
xlabel('Y position (East)');
ylabel('X position (North)');

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

%% Main Control Loop
rho = 5;
jj = 1;
tic;
while (jj < numel(WAYPOINT_x))
    %% recieve updates Pose updates
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
             WAYPOINT_x(jj), WAYPOINT_y(jj), ...
             WAYPOINT_x(jj+1), WAYPOINT_y(jj+1), ...
             rho);

    %% Implement controllers and Conditional Logic
    % PID control for depth
    [depthControlSignal, depthError] = depthPID.update(currentDepth, desiredDepth, dt);

    if abs(depthError) >= 1.5
        %% Update message data with control signal
        [vert_port_thruster_msg, vert_stbd_thruster_msg, ...
            bow_port_thruster_msg, bow_stbd_thruster_msg, ...
            aft_port_thruster_msg, aft_stbd_thruster_msg] = ...
            setDepth(depthControlSignal, stabilityThrust, ...
            vert_port_thruster_msg, vert_stbd_thruster_msg, ...
            bow_port_thruster_msg, bow_stbd_thruster_msg, ...
            aft_port_thruster_msg, aft_stbd_thruster_msg, aft_vert_thruster_msg);
        %% Send message to Fusion
        send(vert_stbd_thruster_pub, vert_stbd_thruster_msg);
        send(vert_port_thruster_pub, vert_port_thruster_msg);
        send(aft_port_thruster_pub, aft_port_thruster_msg);
        send(aft_stbd_thruster_pub, aft_stbd_thruster_msg);
        send(bow_port_thruster_pub, bow_port_thruster_msg);
        send(bow_stbd_thruster_pub, bow_stbd_thruster_msg);
        %send(aft_vert_thruster_pub, aft_vert_thruster_msg);
    else
        sample = toc;
        plotHeadingSample = [plotHeadingSample, sample];

        % PID control for heading
        [headingControlSignal, headingError] = headingPID.updateAngle(currentHeading, desiredHeading, dt);
        if abs(headingError) >= 10 
            rpmAdjustment = 0;
            %% Update Message Data
            [vert_port_thruster_msg, vert_stbd_thruster_msg, ...
                bow_port_thruster_msg, bow_stbd_thruster_msg, ...
                aft_port_thruster_msg, aft_stbd_thruster_msg] = ...
                updateHeading(depthControlSignal, stabilityThrust, headingControlSignal, rpmAdjustment, ... 
                vert_port_thruster_msg, vert_stbd_thruster_msg, ...
                bow_port_thruster_msg, bow_stbd_thruster_msg, ...
                aft_port_thruster_msg, aft_stbd_thruster_msg, aft_vert_thruster_msg);

            %% Send message to Fusion
            send(vert_stbd_thruster_pub, vert_stbd_thruster_msg);
            send(vert_port_thruster_pub, vert_port_thruster_msg);
            send(aft_port_thruster_pub, aft_port_thruster_msg);
            send(aft_stbd_thruster_pub, aft_stbd_thruster_msg);
            send(bow_port_thruster_pub, bow_port_thruster_msg);
            send(bow_stbd_thruster_pub, bow_stbd_thruster_msg);
            %send(aft_vert_thruster_pub, aft_vert_thruster_msg);
        else
            rpmAdjustment = 30;
            %% Update Message Data
            [vert_port_thruster_msg, vert_stbd_thruster_msg, ...
                bow_port_thruster_msg, bow_stbd_thruster_msg, ...
                aft_port_thruster_msg, aft_stbd_thruster_msg] = ...
                updateHeading(depthControlSignal, stabilityThrust, headingControlSignal, rpmAdjustment, ... 
                vert_port_thruster_msg, vert_stbd_thruster_msg, ...
                bow_port_thruster_msg, bow_stbd_thruster_msg, ...
                aft_port_thruster_msg, aft_stbd_thruster_msg, aft_vert_thruster_msg);

            %% Send message to Fusion
            send(vert_stbd_thruster_pub, vert_stbd_thruster_msg);
            send(vert_port_thruster_pub, vert_port_thruster_msg);
            send(aft_port_thruster_pub, aft_port_thruster_msg);
            send(aft_stbd_thruster_pub, aft_stbd_thruster_msg);
            send(bow_port_thruster_pub, bow_port_thruster_msg);
            send(bow_stbd_thruster_pub, bow_stbd_thruster_msg);
            %send(aft_vert_thruster_pub, aft_vert_thruster_msg);
        end

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
    currentYdata = [currentYdata, currentY];
    currentXdata = [currentXdata, currentX];
    set(fusPos, 'Xdata', currentYdata, 'YData', currentXdata);
    drawnow;
    
    %% Evaluate if within parameter to say objective is met
    current_distance_to_next_waypoint = norm([WAYPOINT_x(jj+1) - currentX, WAYPOINT_y(jj+1) - currentY]);
    proximityThreshold = 2;
    if (current_distance_to_next_waypoint < proximityThreshold)
        jj = jj + 1;
    end

    waitfor(rate);
    iterationTime = toc;
end



