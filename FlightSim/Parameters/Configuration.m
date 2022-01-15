function [x_cg, Mass, Vt, alt, config_str, manoeuvre] = Configuration()

%% Setup

% unit conversions
kts = 1.944;    % m/s to knots
ft = 3.2808399; % m to ft

%% Prompt User for Configuration


[userSpeed, userTrainee, manoeuvre] = choosedialog();

Vt = userSpeed/kts;

%% Find the Parameters for the Configuration
if userTrainee == 1
    x_cg    = 0.2922;
    Mass    = 2650 + 110;
    alt     = -500/ft; 
    
    if userSpeed == 100
        Vt      = [100; 0; 0]/kts;
        config_str = 'x_cg = 29.22 %, W = 2760 kg, V_T = 100 kts, alt = 500 ft';
    elseif userSpeed == 300
        Vt      = [300; 0; 0]/kts;
        config_str = 'x_cg = 29.22 %, W = 2760 kg, V_T = 300 kts, alt = 500 ft';
    end
    
elseif userTrainee == 0
    x_cg    = 0.265;
    Mass    = 2650;
    alt     = -500/ft; 

    if userSpeed == 100
        Vt      = [100; 0; 0]/kts;
        config_str = 'x_cg = 26.50 %, W = 2650 kg, V_T = 100 kts, alt = 500 ft';
    elseif userSpeed == 300
        Vt      = [300; 0; 0]/kts;
        config_str = 'x_cg = 26.50 %, W = 2650 kg, V_T = 300 kts, alt = 500 ft';
    end

else
    disp("Invalid Configuation Setting")
    return
end

end