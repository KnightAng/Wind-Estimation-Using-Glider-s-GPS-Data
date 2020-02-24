function [in_profile,n_epochs,ok]= importcsv(filename)
%Read_profile - inputs a motion profile in the following .csv format
% Column 1: time (sec)
% Column 8: norm of the airspeed (m/s)
% Column 9: Latitude (rad)
% Column10: Longitude (rad)
% Column11: altitude (m)
% Column15: groundtrack (rad)
% Column16: vertical velocity of aircraft (m/s)
% Column17: airdensity (kg/m3)
% Column18: thermallift (m/s)
% Column19: roll (rad)

% The followings may be used in the future
% Column 2: ax in body-frame (m/s^2)
% Column 3: ay in body-frame (m/s^2)
% Column 4: az in body-frame (m/s^2)
% Column 5: angular rate roll angle of body w.r.t NED (deg)
% Column 6: angular rate pitch angle of body w.r.t NED (deg)
% Column 7: angular rate yaw angle of body w.r.t NED (deg)
% Column12:14: Magnetometer;

% Inputs:
%   filename     Name of file to write
%
% Outputs:
%   in_profile   Array of data from the file
%   n_epochs     Number of epochs of data in the file
%   ok           Indicates file has the expected number of columns


% Begins
% Read in the profile in .csv format
in_profile = csvread(filename);

% Determine size of file
[n_epochs,n_columns] = size(in_profile);

%Check number of columns is correct (otherwise return)
if n_columns~=19
    disp('Input file has the wrong number of columns')
    ok = false;
else
    ok = true;
    
end