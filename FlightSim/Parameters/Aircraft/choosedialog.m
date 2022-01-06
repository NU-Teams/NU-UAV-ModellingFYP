%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Aircraft:
%   inertia
%       mass
%       moment of inertias
%       centre of mass
%   geometry
%       planform area
%       chord
%       wingspan
%   propulsion
%       P_max
%       efficiency
%   aerodynamic
%       alpha0
%       ... partial coefficient derivatives
%
% Operation:
%   Trimmed flight
%   input impulse
%   
% Environment:
%   fluids
%       sealevel
%       tropopause
%       windSpeed
%   gravity
%   
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function [speed, trainee, manoeuvre] = choosedialog()
    

    box.left = 100;
    box.rght = 800;
    box.btm = 400;
    box.top = 600;
    box.position = [box.left box.btm box.rght-box.left box.top-box.btm];
       
    d = dialog('Position',box.position,'Name','Configuration');
    
	txt1 = uicontrol('Parent',d,'Style','text',...
            'Position',[40 80 150 40],'String','Select a Speed (knots)');
       
    txt2 = uicontrol('Parent',d,'Style','text',...
            'Position',[180 80 150 40],'String','Select a Trainee');
       
    txt3 = uicontrol('Parent',d,'Style','text',...
            'Position',[320 80 150 40],'String','Select a Manoeuvre');
       
       
    popup1 = uicontrol('Parent',d,'Style','popup',...
            'Position',[75 70 100 25],'String',{'100';'300'},...
            'Callback',@popup_callback);
       
    popup2 = uicontrol('Parent',d,'Style','popup',...
            'Position',[215 70 100 25],'String',{'0';'1'},...
            'Callback',@popup_callback2);

    popup3 = uicontrol('Parent',d,'Style','popup',...
            'Position',[355 70 100 25],'String',{'trim';'elevator impulse'; 'aileron impulse'; 'rudder impulse'},...
            'Callback',@popup_callback3);



    btn = uicontrol('Parent',d,...
           'Position',[140 20 70 25],...
           'String','Accept',...
           'Callback','delete(gcf)');
       
    speed = 100;
    trainee = 0;
    manoeuvre = 0;
    
    % Wait for d to close before running to completion
    uiwait(d);
   
    function popup_callback(popup,event)
        idx = popup.Value;
        popup_items = popup.String;
        speed = str2num(char(popup_items(idx,:)));
    end

    function popup_callback2(popup,event)
        idx = popup.Value;
        popup_items = popup.String;
        trainee = str2num(char(popup_items(idx,:)));
    end

    function popup_callback3(popup,event)
        idx = popup.Value;
        popup_items = popup.String;
        man_str = char(popup_items(idx,:));

        if strcmp(man_str, 'trim')
          manoeuvre = 0;
        elseif strcmp(man_str, 'elevator impulse')
          manoeuvre = 1;
        elseif strcmp(man_str, 'aileron impulse')
          manoeuvre = 2;
        elseif strcmp(man_str, 'rudder impulse')
          manoeuvre = 3;
        end

    end
end