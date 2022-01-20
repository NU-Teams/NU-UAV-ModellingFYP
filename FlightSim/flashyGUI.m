%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Output = flashyGUI(data)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function flashyGUI(T, X, X_dot, modelName)

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

simdata = [T' x' y' z' phi' theta' psi'];

% setup a structure from the aero toolbox
animation = Aero.Animation;

% use a aircraft model
%   ac3d_xyzisrgb.ac
%   blueoctagon.ac
%   bluewedge.ac
%   body_xyzisrgb.ac
%   delta2.ac
%   greenarrow.ac
%   pa24-250_orange.ac
%   pa24-250_blue.ac
%   redwedge.ac
%   testrocket.ac
animation.createBody(modelName, 'Ac3d');

% flight data
animation.Bodies{1}.TimeSeriesSource = simdata;

% How the camera moved with the model
animation.Camera.PositionFcn = @staticCameraPosition;

% position of the figure in the pc display
animation.Figure.Position(1) = animation.Figure.Position(1) + 572/2;

% opens the animated figure
animation.show;

% speed multiplier,  shows up in the animation function
animation.FramesPerSecond = 5;

%animation.TimeScaling = 1;

%% ui-Figure
% Open the interactive figure with the gauges
fig = uifigure('Name','Flight Instruments',...
    'Position',[animation.Figure.Position(1)-572 animation.Figure.Position(2)+animation.Figure.Position(4)-602 572 602],...
    'Color',[0.2667 0.2706 0.2784],'Resize','off');

imgPanel = imread('astFlightInstrumentPanel.png');
ax = uiaxes('Parent',fig,'Visible','off','Position',[10 30 530 460],'BackgroundColor',[0.2667 0.2706 0.2784]);
image(ax,imgPanel);
disableDefaultInteractivity(ax);

%% gauges

% Altimeter
altimeter       = uiaeroaltimeter('Parent',fig,'Position',[369 299 144 144]);

% Heading
compassHeading	= uiaeroheading('Parent',fig,'Position',[212 104 144 144]);

% Aispeed Indicator
airspeedIndicator = uiaeroairspeed('Parent',fig,'Position',[56 299 144 144]);
airspeedIndicator.Limits = [20 200];
airspeedIndicator.ScaleColorLimits = [0 50;40 160;160 190;190 200];

% Horizon
hor = uiaerohorizon('Parent',fig,'Position',[212 299 144 144]);

% Climb reader
climbReader = uiaeroclimb('Parent',fig,'Position',[369 104 144 144]);
climbReader.MaximumRate = 8000;

% Turn Coordinate (ie skid ball)
turnCoodinator = uiaeroturn('Parent',fig,'Position',[56 104 144 144]);


%% UI

% Start animation button
button          = uibutton(fig,'state','Text','Start','FontColor','black');
button.Position = [(572-100)/2 502 100 50];

% Time Slider
sl              = uislider('Parent',fig,'Limits',[T(1) T(end)],'FontColor','white');
sl.Position     = [50 60 450 3];

% Change values every timestep
sl.ValueChangingFcn     = @(sl,event)       updateAnimation(fig, T, X, X_dot, animation, event, sl, button);
button.ValueChangedFcn	= @(button,event)	updateAnimation(fig, T, X, X_dot, animation, event, sl, button);

% Slider label
lbl             = uilabel('Parent',fig,'Text',['Time: ' num2str(sl.Value,'%06.2f') ' sec'],'FontColor','white');
lbl.Position	= [(572-100)/2 5 100 30];
end