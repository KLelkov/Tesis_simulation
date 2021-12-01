V = 5.236;
lf = 0.4;
lr = 0.4;
lw = 0.6;
wf = atan(lw / 2/lf);
wr = atan(lw / 2/lr);
Lr = sqrt(lr^2 + 0.25*lw^2);
Lf = sqrt(lf^2 + 0.25*lw^2);

gamma = 0.0799; gamma1 = gamma - gamma^2*lw/(lf+lr); gamma2 = gamma + gamma^2*lw/(lf+lr);

Vxb = 0.25 * ( V*cos(gamma1) + V*cos(gamma2) + V + V);
Vyb = 0.25 * ( V*sin(gamma1) + V*sin(gamma2));
betta = atan(Vyb/Vxb);


dpsi = 0.25*V*sin(gamma1-betta)/Lf/cos(wf+gamma1)-0.25*V*sin(gamma2-betta)/Lf/cos(wf-gamma2) + 0.25*V*sin(betta)/Lr/cos(-wr)+0.25*V*sin(betta)/Lr/cos(wr)
dpsi2 = V*cos(betta)*tan(gamma) / (lf +lr)