function [ quat ] = GetQuatFrom2Vec( u,v )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    m = sqrt(2 + 2 * dot(u,v));
    w = (1/m)* cross(u,v);
    quat = [m/2;w];
end

