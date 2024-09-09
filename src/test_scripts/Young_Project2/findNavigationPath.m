function [navPoints] = findNavigationPath(sv, start, goal, goalPlanner, ind)
    % valDist,...
    %                                  maxIter, maxConDist)
%
% This function plans a path through the defined states space and then
% optimizes the navigation path by iterating through the determined
% navigation points, returning only those points which form the greatest
% straight-line distance between points on the path, until the final point
% is reached
%
% Inputs:   ss          the state space
%
%           sv          the state validator
%           start       starting pose (x,y,z,heading)
%           goal        ending pose (x,y,z,heading)
%           goalPlanner the desired path planner to use
%           ind         the iteration index (used for figure(#))
%
% Output:   navPoints   array of navigation waypoints
%

% goal = [navArray(ind).y, navArray(ind).x, navArray(ind).h];
    % desiredDepth = goal(3);
goalMet = false;
navPoints = [start];

rng(100,'twister') % repeatable result
[pthObj,solnInfo] = plan(goalPlanner,start,goal);

% subplot((length(WAYPOINT_x)-1)/2,(length(WAYPOINT_x)-1)/2,i)
% view([90 -90]);
% set(gca,'XDir','reverse');
figure(ind+4);
show(sv.Map);
hold on;
% Tree expansion
plot(solnInfo.TreeData(:,1),solnInfo.TreeData(:,2),'.-')
% Draw path
plot(pthObj.States(:,1),pthObj.States(:,2),'r-','LineWidth',2)

% Find longest, valid, point-to-point distance along determined path
count = 0; % iteration counter
sStart = 1;                 
while(goalMet == false) && (count < 10) %%% Optimization while loop
                            % count is to prevent an infinite loop
    sLast = 1;
    endStep = [];
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
    % else

    end
    count = count + 1;  % sets break to prevent infinite loop
    
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
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%







end