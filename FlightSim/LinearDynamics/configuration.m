function [x_cg, Mass, Vt, alt, config_str] = configuration()

%% Setup

% unit conversions
kts = 1.944;    % m/s to knots
ft = 3.2808399; % m to ft

%% Prompt User for Configuration
config_str_title = "\nconfiguration()";
config_str1 = "\n1: 100knots w Trainee";
config_str2 = "\n2: 100knots w/o Trainee";
config_str3 = "\n3: 300knots w Trainee";
config_str4 = "\n4: 300knots w/o Trainee";
fprintf(config_str_title)
fprintf(config_str1) 
fprintf(config_str2) 
fprintf(config_str3) 
fprintf(config_str4)

%Prompt Config
prompt = "\n What Configuraation Would you like to use? (1-4)\n";
% Recieve config from Command Window
config = input(prompt);

%% Find the Parameters for the Configuration
if config == 1
    %   Inga - 100knots w Trainee
    x_cg    = 0.2922;
    Mass    = 2650+110;
    Vt      = 100/kts;
    alt     = -500/ft; 
    config_str = 'x_cg = 29.22 %, W = 2760 kg, V_T = 100 kts, alt = 500 ft';
elseif config == 2
    %   Ash - 100knots w/o Trainee
    x_cg    = 0.265;
    Mass    = 2650;
    Vt      = 100/kts;
    alt     = -500/ft; 
    config_str = 'x_cg = 26.50 %, W = 2650 kg, V_T = 100 kts, alt = 500 ft';
elseif config == 3
    %   Jason - 300knots w Trainee
    x_cg = 0.2922;
    Mass = 2650+110;
    Vt = 300/kts;
    alt = -500/ft; 
    config_str = 'x_cg = 29.22 %, W = 2760 kg, V_T = 300 kts, alt = 500 ft';
elseif config == 4
    %   Marty - 300knots w/o Trainee
    
    % Initialising Independant Vars
    x_cg = 0.265;
    Mass = 2650;
    Vt = 300/kts;
    alt = -500/ft;   
    config_str = 'x_cg = 26.50 %, W = 2760 kg, V_T = 100 kts, alt = 500 ft';
else
    disp("Invalid Configuation Setting")
    return
end

end