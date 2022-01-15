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
function [densityRatio, dynamicPressure] = FlowProperties(X_k, ENVIRONMENT)
%% CODE

g                     = ENVIRONMENT.gravity;                % Gravity
R_gas_const           = ENVIRONMENT.universalGasConst;   	% Universal gas constant
LapseRate             = ENVIRONMENT.lapseRate;              % Lapse Rate
density_sl            = ENVIRONMENT.Sealevel.density;       % sea level air density
Temp_sl               = ENVIRONMENT.Sealevel.temperature;   % Temp. at sea-level (K)
density_tropopause    = 0.3636;       % tropopause density
altitude_tropopause   = 11000;        % tropopause alt. (m)
Temp_tropopause       = 217;          % Temp. at tropopause (K)

% Initialise True-Airspeed
[VT, ~, ~] = AeroAngles(X_k);

% Initialise Altitude
z = abs(X_k(13));

% Initialise Temperature
Temperature = ones(size(z));

% Temperature below the tropopause
Temperature(z<altitude_tropopause) = Temp_sl - LapseRate.*z(z<altitude_tropopause);
% Temperature above the tropopause
Temperature(z>=altitude_tropopause) = Temp_tropopause;


% Initialise density Ratio
densityRatio = ones(size(z));

% Density below tropopause
densityRatio(z<altitude_tropopause) = (Temperature(z<altitude_tropopause)./Temp_sl).^(g./(LapseRate.*R_gas_const)-1);
% Density above tropopause
densityRatio(z>=altitude_tropopause) =(density_tropopause/density_sl).*exp(-g.*(z(z>=altitude_tropopause)-altitude_tropopause)./(Temperature(z>=altitude_tropopause).*R_gas_const));


% Dynamic Pressure
dynamicPressure = (1/2) *(densityRatio*density_sl) *(VT.^2);

end