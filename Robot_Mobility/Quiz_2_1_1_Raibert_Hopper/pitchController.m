function Tphi = pitchController(phi,phiDesired,dphi_dt)
kd_phi = 5;
kp_phi = 4;
Tphi= -kd_phi*dphi_dt-kp_phi*(phi-pi/12);