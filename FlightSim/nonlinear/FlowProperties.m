%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% [density_ratio, dynamic_pressure] = FlowProperties(X_k)
% Returns the density ratio for a given altitude and the dynamic pressure
% for a TRUE airspeed
%
% CALLED FUNCTIONS:
%   AeroAngles(X_k)
%   
% Jason Iredale, 30/04/2021
% Jason Iredale, 7/05/2021 0110
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [density_ratio, dynamic_pressure] = FlowProperties(X_k)
%% CODE

g                     = 9.81;         % Gravity
R_gas_const           = 287;          % Universal gas constant
LapseRate             = 6.5*10^-3;    % Lapse Rate
density_sl            = 1.225;        % sea level air density
Temp_sl               = 288.16;       % Temp. at sea-level (K)
density_tropopause    = 0.3636;       % tropopause density
altitude_tropopause   = 11000;        % tropopause alt. (m)
Temp_tropopause       = 217;          % Temp. at tropopause (K)

% Initialise True-Airspeed
[true_airspeed, ~, ~] = AeroAngles(X_k);

% Initialise Altitude
z = abs(X_k(13));

% Initialise Temperature
Temperature = ones(size(z));

% Temperature below the tropopause
Temperature(z<altitude_tropopause) = Temp_sl - LapseRate.*z(z<altitude_tropopause);
% Temperature above the tropopause
Temperature(z>=altitude_tropopause) = Temp_tropopause;


% Initialise density Ratio
density_ratio = ones(size(z));

% Density below tropopause
density_ratio(z<altitude_tropopause) = (Temperature(z<altitude_tropopause)./Temp_sl).^(g./(LapseRate.*R_gas_const)-1);
% Density above tropopause
density_ratio(z>=altitude_tropopause) =(density_tropopause/density_sl).*exp(-g.*(z(z>=altitude_tropopause)-altitude_tropopause)./(Temperature(z>=altitude_tropopause).*R_gas_const));


% Dynamic Pressure
dynamic_pressure = (1/2) *(density_ratio*density_sl) *(true_airspeed.^2);

end