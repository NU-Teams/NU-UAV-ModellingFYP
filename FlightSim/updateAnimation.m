function updateAnimation(uifigure, T, X, X_dot, animation, event, sl, button)
% ASTHELPERFLIGHTINSTRUMENTSANIMATION Helper function for the flight
% instruments example. This helper updates the values of the flight
% instruments and animation according to the time selected in the slider
% component.
%
% Copyright 2018 The MathWorks, Inc.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Unpack
u   = X(1,:);
v   = X(2,:);
w   = X(3,:);
p   = X(4,:);
q   = X(5,:);
r   = X(6,:);
phi = X(7,:);
theta=X(8,:);
psi = X(9,:);
x   = X(10,:);
y   = X(11,:);
z   = X(12,:);

VT      = X(13,:);
beta    = X(14,:);
alpha   = X(15,:);

dv      = X_dot(2, :);
dw      = X_dot(3, :);
dz      = X_dot(13,:);

f = animation.FramesPerSecond;
func = rateControl(f);
% Obtain current time value from the slider object
t = event.Value;
if button.Value == 1
    button.Text = 'Stop';
    t=sl.Value;
else
    button.Text = 'Start';
end

%% update gauges

% Get all children from loaded figure
child               = uifigure.Children;

% Get altimeter object
altimeter           = findobj(child,'Type','uiaeroaltimeter');

% Get airspeed indicator object
airspeedIndicator	= findobj(child,'Type','uiaeroairspeed');

% Get heading indicator object
compassHeading      = findobj(child,'Type','uiaeroheading');

% Get artificial horizon object
hor                 = findobj(child,'Type','uiaerohorizon');

% Get climb rate indicator object
climbReader         = findobj(child,'Type','uiaeroclimb');

% Get turn coordinator object
turnCoodinator      = findobj(child,'Type','uiaeroturn');


%% update label   

% Update the displayed time of the animation 
lbl         = findobj(child,'Type','uilabel');
lbl.Text	= ['Time: ' num2str(t,'%06.2f') ' sec'];


%% run animation

% Get the index of the time
T_idx       = find(T<=t);
T_idx       = T_idx(end);
T_length	= length(T);
DT          = T(2)-T(1);

reset(func);
% run in a loop throughout the time array
for T_idx = T_idx:(1/(animation.FramesPerSecond*DT)):T_length
    
    % if the user turns presses the button to turn off the animation
    if button.Value == 0
        break
    end
    
    % update altitude
    altimeter.Altitude      = convlength(-z(T_idx), 'm', 'ft');

    % update body heading from north
    compassHeading.Heading	= convang(psi(T_idx),'rad','deg');

    % update roll and pitch
    hor.Roll                = convang(phi(T_idx),'rad','deg');
    hor.Pitch               = convang(theta(T_idx),'rad','deg');

    % Estimate speeds (todo: we don't need to estimate)
    if T_idx > 1
        
        % Set climb rate
        climbReader.ClimbRate       = convvel(-dz(T_idx),'m/s','ft/min');
        
        % Set airspeed
        airspeedIndicator.Airspeed	= convvel(VT(T_idx),'m/s','kts');
        
        % Set turn coordinator
        turnCoodinator.Turn         = convangvel(r(T_idx),'rad/s','deg/s');
        
        g_y                         = 9.81*sin(phi(T_idx))*cos(theta(T_idx));
        g_z                         = 9.81*cos(phi(T_idx))*cos(theta(T_idx));
        slip                        = atan((g_y-dv(T_idx))/(g_z-dw(T_idx)));
        turnCoodinator.Slip         = convang(slip,'rad','deg');
        
        % if we reach the end of the animation
        if T_idx == T_length
            button.Value	= 0;
            button.Text     = 'Start';
            sl.Value        = 0;
            break
        end
        
    % else we are at the beginning of the animation, set everything to zero
    else
        climbReader.ClimbRate       = 0;
        airspeedIndicator.Airspeed	= 0;
        turnCoodinator.Slip         = 0;
        turnCoodinator.Turn         = 0;
    end

    % Update animation object with current time
    animation.updateBodies(T(T_idx));
    animation.updateCamera(T(T_idx));
    
    % update time label and the slider
    t = T(T_idx);
    sl.Value = t;
    lbl.Text = ['Time: ' num2str(t,'%06.2f') ' sec'];
    
    waitfor(func);
    func.TotalElapsedTime;
end

end
