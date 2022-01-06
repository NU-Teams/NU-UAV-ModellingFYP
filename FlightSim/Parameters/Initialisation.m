%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% [ FD ] = Initialisation()
% This function generates and returns the flight data structure which
% defines the aircraft data and aerodynamic derivatives.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [aircraft, environment, config_str] = Initialisation()





[x_cg, mass, VG, alt, config_str, manoeuvre] = Configuration();

aircraft = AircraftParameters(x_cg, mass);

aircraft.x_cg = x_cg;
aircraft.alt = alt;
aircraft.manoeuvre = manoeuvre;
aircraft.VG_e     = VG;
aircraft.bearing  = atan(aircraft.VG_e(2)./aircraft.VG_e(1));


environment.fluid = AirFlowParameters();


aircraft.VW_e     = environment.fluid.windSpeed*[cos(environment.fluid.windBearing); sin(environment.fluid.windBearing); 0];

aircraft.VT_e = aircraft.VG_e + aircraft.VW_e;
aircraft.VT = norm(aircraft.VT_e);
aircraft.beta = asin(aircraft.VT_e(2)./aircraft.VT);

end
