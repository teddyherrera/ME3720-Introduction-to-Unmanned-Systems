function [binMap] = fillOccupancy(binMap, resolution, inflateVal, minX, maxX)
%
% use to fill in the occupied spaces in the binary matrix

for i = -3:(1/resolution):4
    for j = -3:(1/resolution):4
        setOccupancy(binMap,[i,j],1);  % Defines area of BOP
    end
end

if inflateVal > 0
    binMap.inflate(inflateVal);
end

for i = minX:(1/resolution):maxX      % Defines edges
    setOccupancy(binMap,[i,minX],1);
    setOccupancy(binMap,[i,maxX],1);
    setOccupancy(binMap,[minX,i],1);
    setOccupancy(binMap,[maxX,i],1);
end

end