function fluid = AirFlowParameters()

fluid.R_gas_const           = 287;          % Universal gas constant
fluid.LapseRate             = 6.5*10^-3;    % Lapse Rate

fluid.sealevel.altitude     = 0;
fluid.seaLevel.density      = 1.225;        % sea level air density
fluid.sealevel.temperature  = 288.16;       % Temp. at sea-level (K)

fluid.tropopause.altitude   = 11000;        % tropopause alt. (m)
fluid.tropopause.density    = 0.3636;       % tropopause density
fluid.tropopause.temperature= 217;          % Temp. at tropopause (K)

kts = 1.944;
fluid.windSpeed = 0/kts;

deg = 180/pi;
fluid.windBearing = 20/deg;   % A northernly wind is 0 degrees

end