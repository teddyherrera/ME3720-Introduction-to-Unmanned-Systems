function [navMap, ss, sv] = findNavigationMap(ss, mapMat, xMin, yMin, resolution, valDist)%,worldSize, inflation)
%
% Creates an occupancy map centered on the world origin
% Inputs:   ss          the state space
%           mapMat      binary matrix defining map free space and obstacles
%           xMin        minimum x boundary }used to shift the map origin
%           yMin        minimum y boundary } to the world center
%           worldSize   size of the world in meters
%           resolution  map resolution (cells/meter)
%           inflation   amount by which to inflate the map obstacles
%
% Outputs:  sv          state validator
%           navMap      
%
%
sv = validatorOccupancyMap(ss);
sv.ValidationDistance = valDist;
navMap = occupancyMap(mapMat, resolution);
navMap.GridLocationInWorld = [xMin, yMin];
% if inflation > 0
%     navMap.inflate(inflation);
% end
ss.StateBounds = [navMap.XWorldLimits; navMap.YWorldLimits; [-pi pi]];
sv.Map = navMap;


end