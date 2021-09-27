%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% [ FD ] = Initialisation()
% This function generates and returns the flight data structure which
% defines the aircraft data and aerodynamic derivatives.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [FD, config_str] = Initialisation()
% Units
kts = 1.944;
deg = 180/pi;

[x_cg, mass, VG, alt, config_str] = Configuration();

[ FD ] = AircraftParameters(x_cg, mass);

FD.x_cg = x_cg;
FD.alt = alt;

FD.VG_e     = VG;
FD.bearing  = atan(FD.VG_e(2)./FD.VG_e(1));
windSpeed   = 0/kts;
windBearing = 20/deg;   % A northernly wind is 0 degrees
FD.VW_e     = windSpeed*[cos(windBearing); sin(windBearing); 0];

FD.VT_e = FD.VG_e + FD.VW_e;
FD.VT = norm(FD.VT_e);
FD.beta = asin(FD.VT_e(2)./FD.VT);

end
