function MilSpec = PlotMilSpec(X_bar, AIRCRAFT, ENVIRONMENT)

%%UNPACK

CLa = AIRCRAFT.Aero.CLa;
S   = AIRCRAFT.Geom.S;
m	= AIRCRAFT.Inertia.m;

g   = ENVIRONMENT.gravity;


[~, ~,~, Wn,zeta] = eig_analysis(AIRCRAFT.A);

Wn_SP	= Wn(3:4);
Wn_LP	= Wn(5:6);
Wn_Alt  = Wn(10);

Wn_Roll = Wn(7);
Wn_DR   = Wn(8:9);
Wn_SR   = Wn(11);

zeta_SP	= zeta(3:4);
zeta_LP	= zeta(5:6);
zeta_Alt= zeta(10);

zeta_Roll=zeta(7);
zeta_DR = zeta(8:9);
zeta_SR = zeta(11);


[~, Q]	= FlowProperties(X_bar, ENVIRONMENT);
W       = m*g;

dn_dalpha = CLa*Q*S/W;


for i = 1:4
    
    % Category A
    % Level 1
    % dn_dalpha
    Class(i).Cat(1).SP.Dndalpha.lvl(1).lwr.k = 0.517;
    Class(i).Cat(1).SP.Dndalpha.lvl(1).lwr.n = 0.504;
    Class(i).Cat(1).SP.Dndalpha.lvl(1).lwr.y = 1;
    Class(i).Cat(1).SP.Dndalpha.lvl(1).upr.k = 1.895;
    Class(i).Cat(1).SP.Dndalpha.lvl(1).upr.n = 0.501;
    Class(i).Cat(1).SP.Dndalpha.lvl(1).upr.y = NaN;
    Class(i).Cat(1).SP.Dndalpha.lvl(1).lft.x = 0;
    % dampRatio
    Class(i).Cat(1).SP.dampRatio.lvl(1).region = semilogXcircle((log10(1.3)-log10(0.35))/2, [(log10(1.3)+log10(0.35))/2, 2.40], 2.0, 1.8);
    %DR
    Class(i).Cat(1).DR.lvl(1).zeta = 0.19;
    Class(i).Cat(1).DR.lvl(1).zetaWn = 0.35;
    Class(i).Cat(1).DR.lvl(1).Wn = 1;
    
    % Level 2
    % dn_dalpha
    Class(i).Cat(1).SP.Dndalpha.lvl(2).lwr.k = 0.404;
    Class(i).Cat(1).SP.Dndalpha.lvl(2).lwr.n = 0.498;
    Class(i).Cat(1).SP.Dndalpha.lvl(2).lwr.y = 0.6;
    Class(i).Cat(1).SP.Dndalpha.lvl(2).upr.k = 3.160;
    Class(i).Cat(1).SP.Dndalpha.lvl(2).upr.n = 0.505;
    Class(i).Cat(1).SP.Dndalpha.lvl(2).upr.y = NaN;
    Class(i).Cat(1).SP.Dndalpha.lvl(2).lft.x = 0;
    % dampRatio
    Class(i).Cat(1).SP.dampRatio.lvl(2).region = semilogXcircle((log10(2.0)-log10(0.25))/2, [(log10(2.0)+log10(0.25))/2, 2.45], 2.3, 1.6);
    %DR
    Class(i).Cat(1).DR.lvl(2).zeta = 0.02;
    Class(i).Cat(1).DR.lvl(2).zetaWn = 0.05;
    Class(i).Cat(1).DR.lvl(2).Wn = 0.4;
    
    % Level 3
    % dn_dalpha
    Class(i).Cat(1).SP.Dndalpha.lvl(3).lwr.k = 0.404;
    Class(i).Cat(1).SP.Dndalpha.lvl(3).lwr.n = 0.498;
    Class(i).Cat(1).SP.Dndalpha.lvl(3).lwr.y = 0;
    Class(i).Cat(1).SP.Dndalpha.lvl(3).upr.k = NaN;
    Class(i).Cat(1).SP.Dndalpha.lvl(3).upr.n = NaN;
    Class(i).Cat(1).SP.Dndalpha.lvl(3).upr.y = NaN;
    Class(i).Cat(1).SP.Dndalpha.lvl(3).lft.x = 0;
    % dampRatio
    Class(i).Cat(1).SP.dampRatio.lvl(3).region = semilogXcircle((log10(3.0)-log10(0.15))/2, [(log10(3.0)+log10(0.15))/2, 2.50], 2.3, 1.6);
    %DR
    Class(i).Cat(1).DR.lvl(3).zeta = 0;
    Class(i).Cat(1).DR.lvl(3).zetaWn = 0;
    Class(i).Cat(1).DR.lvl(3).Wn = 0.4;
    
    
    % Category B
    % Level 1
    % dn_dalpha
    Class(i).Cat(2).SP.Dndalpha.lvl(1).lwr.k = 0.293;
    Class(i).Cat(2).SP.Dndalpha.lvl(1).lwr.n = 0.485;
    Class(i).Cat(2).SP.Dndalpha.lvl(1).lwr.y = 0;
    Class(i).Cat(2).SP.Dndalpha.lvl(1).upr.k = 1.849;
    Class(i).Cat(2).SP.Dndalpha.lvl(1).upr.n = 0.511;
    Class(i).Cat(2).SP.Dndalpha.lvl(1).upr.y = NaN;
    Class(i).Cat(2).SP.Dndalpha.lvl(1).lft.x = 0;
    % dampRatio
    Class(i).Cat(2).SP.dampRatio.lvl(1).region = semilogXcircle((log10(2.0)-log10(0.30))/2, [(log10(2.0)+log10(0.30))/2, 2.40], 2.0, 1.8);
    %DR
    Class(i).Cat(2).DR.lvl(1).zeta = 0.08;
    Class(i).Cat(2).DR.lvl(1).zetaWn = 0.15;
    Class(i).Cat(2).DR.lvl(1).Wn = 0.4;
    
    % Level 2
    Class(i).Cat(2).SP.Dndalpha.lvl(2).lwr.k = 0.200;
    Class(i).Cat(2).SP.Dndalpha.lvl(2).lwr.n = 0.500;
    Class(i).Cat(2).SP.Dndalpha.lvl(2).lwr.y = 0;
    Class(i).Cat(2).SP.Dndalpha.lvl(2).upr.k = 3.106;
    Class(i).Cat(2).SP.Dndalpha.lvl(2).upr.n = 0.505;
    Class(i).Cat(2).SP.Dndalpha.lvl(2).upr.y = NaN;
    Class(i).Cat(2).SP.Dndalpha.lvl(2).lft.x = 0;
    % dampRatio
    Class(i).Cat(2).SP.dampRatio.lvl(2).region = semilogXcircle((log10(2.0)-log10(0.20))/2, [(log10(2.0)+log10(0.20))/2, 2.45], 2.3, 1.6);
    %DR
    Class(i).Cat(2).DR.lvl(2).zeta = 0.02;
    Class(i).Cat(2).DR.lvl(2).zetaWn = 0.05;
    Class(i).Cat(2).DR.lvl(2).Wn = 0.4;
    
    % Level 3
    Class(i).Cat(2).SP.Dndalpha.lvl(3).lwr.k = 0.200;
    Class(i).Cat(2).SP.Dndalpha.lvl(3).lwr.n = 0.500;
    Class(i).Cat(2).SP.Dndalpha.lvl(3).lwr.y = 0;
    Class(i).Cat(2).SP.Dndalpha.lvl(3).upr.k = NaN;
    Class(i).Cat(2).SP.Dndalpha.lvl(3).upr.n = NaN;
    Class(i).Cat(2).SP.Dndalpha.lvl(3).upr.y = NaN;
    Class(i).Cat(2).SP.Dndalpha.lvl(3).lft.x = 0;
    % dampRatio
    Class(i).Cat(2).SP.dampRatio.lvl(3).region = semilogXcircle((log10(3.0)-log10(0.15))/2, [(log10(3.0)+log10(0.15))/2, 2.50], 2.3, 1.6);
    %DR
    Class(i).Cat(2).DR.lvl(3).zeta = 0;
    Class(i).Cat(2).DR.lvl(3).zetaWn = 0;
    Class(i).Cat(2).DR.lvl(3).Wn = 0.4;
    
    
    % Category C
    % Level 1
    % dn_dalpha
    Class(i).Cat(3).SP.Dndalpha.lvl(1).lwr.k = 0.404;
    Class(i).Cat(3).SP.Dndalpha.lvl(1).lwr.n = 0.498;
    Class(i).Cat(3).SP.Dndalpha.lvl(1).lwr.y = 0;
    Class(i).Cat(3).SP.Dndalpha.lvl(1).upr.k = 1.849;
    Class(i).Cat(3).SP.Dndalpha.lvl(1).upr.n = 0.511;
    Class(i).Cat(3).SP.Dndalpha.lvl(1).upr.y = NaN;
    Class(i).Cat(3).SP.Dndalpha.lvl(1).lft.x = 0;
    % dampRatio
    Class(i).Cat(3).SP.dampRatio.lvl(1).region = semilogXcircle((log10(1.3)-log10(0.35))/2, [(log10(1.3)+log10(0.35))/2, 2.40], 2.0, 1.8);
    %DR
    Class(i).Cat(3).DR.lvl(1).zeta = 0.08;
    Class(i).Cat(3).DR.lvl(1).zetaWn = 0.15;
    Class(i).Cat(3).DR.lvl(1).Wn = 1;
    
    % Level 2
    Class(i).Cat(3).SP.Dndalpha.lvl(2).lwr.k = 0.320;
    Class(i).Cat(3).SP.Dndalpha.lvl(2).lwr.n = 0.486;
    Class(i).Cat(3).SP.Dndalpha.lvl(2).lwr.y = 0;
    Class(i).Cat(3).SP.Dndalpha.lvl(2).upr.k = 3.058;
    Class(i).Cat(3).SP.Dndalpha.lvl(2).upr.n = 0.515;
    Class(i).Cat(3).SP.Dndalpha.lvl(2).upr.y = NaN;
    Class(i).Cat(3).SP.Dndalpha.lvl(2).lft.x = 0;
    % dampRatio
    Class(i).Cat(3).SP.dampRatio.lvl(2).region = semilogXcircle((log10(2.0)-log10(0.25))/2, [(log10(2.0)+log10(0.25))/2, 2.45], 2.3, 1.6);
    %DR
    Class(i).Cat(3).DR.lvl(2).zeta = 0.02;
    Class(i).Cat(3).DR.lvl(2).zetaWn = 0.05;
    Class(i).Cat(3).DR.lvl(2).Wn = 0.4;
    
    % Level 3
    Class(i).Cat(3).SP.Dndalpha.lvl(3).lwr.k = 0.320;
    Class(i).Cat(3).SP.Dndalpha.lvl(3).lwr.n = 0.486;
    Class(i).Cat(3).SP.Dndalpha.lvl(3).lwr.y = 0;
    Class(i).Cat(3).SP.Dndalpha.lvl(3).upr.k = NaN;
    Class(i).Cat(3).SP.Dndalpha.lvl(3).upr.n = NaN;
    Class(i).Cat(3).SP.Dndalpha.lvl(3).upr.y = NaN;
    Class(i).Cat(3).SP.Dndalpha.lvl(3).lft.x = 0;
    % dampRatio
    Class(i).Cat(3).SP.dampRatio.lvl(3).region = semilogXcircle((log10(3.0)-log10(0.15))/2, [(log10(3.0)+log10(0.15))/2, 2.50], 2.3, 1.6);
    %DR
    Class(i).Cat(3).DR.lvl(3).zeta = 0;
    Class(i).Cat(3).DR.lvl(3).zetaWn = 0;
    Class(i).Cat(3).DR.lvl(3).Wn = 0.4;

end

Class(1).Cat(3).SP.Dndalpha.lvl(1).lwr.y = 0.9;
Class(1).Cat(3).SP.Dndalpha.lvl(1).lft.x = 2.5;

Class(1).Cat(3).SP.Dndalpha.lvl(2).lwr.y = 0.6;
Class(1).Cat(3).SP.Dndalpha.lvl(2).lft.x = 1.5;

Class(1).Cat(3).SP.Dndalpha.lvl(3).lwr.y = 0;
Class(1).Cat(3).SP.Dndalpha.lvl(3).lft.x = 0;


Class(2).Cat(3).SP.Dndalpha.lvl(1).lwr.y = 0.7;
Class(2).Cat(3).SP.Dndalpha.lvl(1).lft.x = 2.0;

Class(2).Cat(3).SP.Dndalpha.lvl(2).lwr.y = 0.4;
Class(2).Cat(3).SP.Dndalpha.lvl(2).lft.x = 1.0;

Class(2).Cat(3).SP.Dndalpha.lvl(3).lwr.y = 0;
Class(2).Cat(3).SP.Dndalpha.lvl(3).lft.x = 0;

Class(2).Cat(1).DR.lvl(1).zeta = 0.19;
Class(2).Cat(1).DR.lvl(1).zetaWn = 0.35;
Class(2).Cat(1).DR.lvl(1).Wn = 0.4;

Class(2).Cat(3).DR.lvl(1).zeta = 0.08;
Class(2).Cat(3).DR.lvl(1).zetaWn = 0.1;
Class(2).Cat(3).DR.lvl(1).Wn = 0.4;


Class(3).Cat(3).SP.Dndalpha.lvl(1).lwr.y = 0.7;
Class(3).Cat(3).SP.Dndalpha.lvl(1).lft.x = 2.0;

Class(3).Cat(3).SP.Dndalpha.lvl(2).lwr.y = 0.4;
Class(3).Cat(3).SP.Dndalpha.lvl(2).lft.x = 1.0;

Class(3).Cat(3).SP.Dndalpha.lvl(3).lwr.y = 0;
Class(3).Cat(3).SP.Dndalpha.lvl(3).lft.x = 0;

Class(3).Cat(1).DR.lvl(1).zeta = 0.19;
Class(3).Cat(1).DR.lvl(1).zetaWn = 0.35;
Class(3).Cat(1).DR.lvl(1).Wn = 0.4;

Class(3).Cat(3).DR.lvl(1).zeta = 0.08;
Class(3).Cat(3).DR.lvl(1).zetaWn = 0.1;
Class(3).Cat(3).DR.lvl(1).Wn = 0.4;


Class(4).Cat(3).SP.Dndalpha.lvl(1).lwr.y = 0.9;
Class(4).Cat(3).SP.Dndalpha.lvl(1).lft.x = 2.5;

Class(4).Cat(3).SP.Dndalpha.lvl(2).lwr.y = 0.6;
Class(4).Cat(3).SP.Dndalpha.lvl(2).lft.x = 1.5;

Class(4).Cat(3).SP.Dndalpha.lvl(3).lwr.y = 0;
Class(4).Cat(3).SP.Dndalpha.lvl(3).lft.x = 0;


% Package output
MilSpec.dn_dalpha   = dn_dalpha;
MilSpec.dampRatio   = zeta;
MilSpec.natFreq     = Wn;

MilSpec.Class       = Class;



end