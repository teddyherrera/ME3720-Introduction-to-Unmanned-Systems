function [vertPortThrusterMsg, vertStbdThrusterMsg, ...
    bowPortThrusterMsg, bowStbdThrusterMsg, ...
    aftPortThrusterMsg, aftStbdThrusterMsg, aftVertThrusterMsg] = ...
    setDepth(depthControlSignal, stabilityThrust, ... 
    vertPortThrusterMsg, vertStbdThrusterMsg, ...
    bowPortThrusterMsg, bowStbdThrusterMsg,...
    aftPortThrusterMsg, aftStbdThrusterMsg, aftVertThrusterMsg)

    % assign maximum RPM limits
    MaxRPM = 1200;

    % Apply constraints to thruster commands to be within +/- 1200 RPM
    vertPortCommand = max(min(-depthControlSignal , MaxRPM), -MaxRPM);  % Positive RPMs send down
    vertStbdCommand = max(min(depthControlSignal, MaxRPM), -MaxRPM);  % Positive RPMs send up

    bowPortCommand = -stabilityThrust;  % Negative for clockwise due to positive RPMs send CCW
    bowStbdCommand = stabilityThrust;  % Positive sends CCW, need negative for CW
    aftPortCommand = -stabilityThrust;
    aftStbdCommand = stabilityThrust;

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

