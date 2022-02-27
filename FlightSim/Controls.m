function U = controls(AIRCRAFT, OPERATION, Ubar)

Delta_U = OPERATION.manualControls;
U = Delta_U + Ubar.*ones(5,length(Delta_U(1,:)));

delta_T = U(1,:);
delta_e = U(2,:);
delta_a = U(3,:);
delta_r = U(4,:);
delta_f = U(5,:);

delta_T(delta_T<AIRCRAFT.CtrlLim.Lwr(1)) = AIRCRAFT.CtrlLim.Lwr(1);
delta_e(delta_e<AIRCRAFT.CtrlLim.Lwr(2)) = AIRCRAFT.CtrlLim.Lwr(2);
delta_a(delta_a<AIRCRAFT.CtrlLim.Lwr(3)) = AIRCRAFT.CtrlLim.Lwr(3);
delta_r(delta_r<AIRCRAFT.CtrlLim.Lwr(4)) = AIRCRAFT.CtrlLim.Lwr(4);
delta_f(delta_f<AIRCRAFT.CtrlLim.Lwr(5)) = AIRCRAFT.CtrlLim.Lwr(5);

delta_T(delta_T>AIRCRAFT.CtrlLim.Upr(1)) = AIRCRAFT.CtrlLim.Upr(1);
delta_e(delta_e>AIRCRAFT.CtrlLim.Upr(2)) = AIRCRAFT.CtrlLim.Upr(2);
delta_a(delta_a>AIRCRAFT.CtrlLim.Upr(3)) = AIRCRAFT.CtrlLim.Upr(3);
delta_r(delta_r>AIRCRAFT.CtrlLim.Upr(4)) = AIRCRAFT.CtrlLim.Upr(4);
delta_f(delta_f>AIRCRAFT.CtrlLim.Upr(5)) = AIRCRAFT.CtrlLim.Upr(5);

U = [delta_T;
     delta_e;
     delta_a;
     delta_r;
     delta_f];
end
