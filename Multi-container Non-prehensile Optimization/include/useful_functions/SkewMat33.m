function [ skew_mat ] = SkewMat33(v)
% SKEW_MAT33  Get [3x3] skew symmetrical matrix of a vector in R3
%	Institute of Robotics
%	Stefan Gadringer, c2018
% 	skew_mat = SkewMat33(v)
%
%	Input:
%	v...........Vector in R3
%
%	Output:
%	skew_mat....Skew symmetrical matrix [3x3]

  skew_mat = [0, -  v(3), v(2);
              v(3), 0,    -v(1);
             -v(2), v(1), 0];
end