function [vertPortThrusterMsg, vertStbdThrusterMsg, ...
    bowPortThrusterMsg, bowStbdThrusterMsg, ...
    aftPortThrusterMsg, aftStbdThrusterMsg, aftVertThrusterMsg] = ...
    updateHeading(depthControlSignal, stabilityThrust, headingControlSignal, rpmAdjustment,... 
    vertPortThrusterMsg, vertStbdThrusterMsg, ...
    bowPortThrusterMsg, bowStbdThrusterMsg,...
    aftPortThrusterMsg, aftStbdThrusterMsg, aftVertThrusterMsg)

    % assign maximum RPM limits
    MaxRPM_Depth = 400;
    headingControlSignal = max(min(headingControlSignal, 25), -25);

    % Apply constraints to thruster commands to be within +/- 1200 RPM
    vertPortCommand = max(min(-depthControlSignal , MaxRPM_Depth), -MaxRPM_Depth);  % Positive RPMs send down
    vertStbdCommand = max(min(depthControlSignal, MaxRPM_Depth), -MaxRPM_Depth);  % Positive RPMs send up

    bowPortCommand = -stabilityThrust + headingControlSignal;  % Negative for clockwise due to positive RPMs send CCW
    bowStbdCommand = stabilityThrust + headingControlSignal ;  % Positive sends CCW, need negative for CW
    aftPortCommand = -stabilityThrust + rpmAdjustment;
    aftStbdCommand = stabilityThrust - rpmAdjustment;

    %aftVertCommand = max(min(pitchControlSignal, MaxRPM_rotational), -MaxRPM_rotational);

    % Set the command in the message data
    vertPortThrusterMsg.Data = vertPortCommand;
    vertStbdThrusterMsg.Data = vertStbdCommand;
    bowPortThrusterMsg.Data = bowPortCommand; 
    bowStbdThrusterMsg.Data = bowStbdCommand;
    aftPortThrusterMsg.Data = aftPortCommand;
    aftStbdThrusterMsg.Data = aftStbdCommand;
   % aftVertThrusterMsg.Data = aftVertCommand;
end

