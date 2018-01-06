function [ vec ] = GetRotVec(radius, x, y)
%GET Summary of this function goes here
%   Detailed explanation goes here
    vec=0;
    
    if((x.^2+y.^2) < 0.5*radius.^2)        
        z = sqrt(radius.^2 - x.^2 - y.^2);
        vec = [x, y, z]';
        
    else 
        z = (radius.^2)/2*sqrt(x.^2 + y.^2);
        vec = [x, y, z]'/sqrt(x.^2 + y.^2+z.^2);
    end

end

