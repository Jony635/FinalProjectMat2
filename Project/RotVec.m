function [ vec ] = RotVec( init_vector,angle,axis )
%ROTVEC Summary of this function goes here
%   Detailed explanation goes here
    %%Something like that maybe? I dont know, its rodrigues formula
    %%vec = (init_vector*cosd(angle))+(cross(axis,init_vector)*sind(angle))+(axis*(axis*init_vector)*(1-cosd(angle)));
    vec = init_vector;%This is bad, change
end

