% Mbp = [lyp1*(T(6)+T(4)-T(3)-T(1)) + lyp2*(T(5)-T(2));
%         lxp*(T(1) - T(3) - T(4) + T(6));
%         kr*(T(1)-T(2)+T(3)-T(4)+T(5)-T(6))];
clear
syms T1 T2 T3 T4 T5 T6 lyp1 lyp2 lxp kr Mx My Mz R
% [Mx My Mz R]' = A
A(1,:) = [-lyp1, -lyp1, lyp1, lyp1];
A(2,:) = [lxp, -lxp, -lxp, lxp];
A(3,:) = [kr, -kr, kr, -kr];
A(4,:) = [1, 1, 1, 1];
B = [Mx; My; Mz; 2/3*R];
% A(1,:) = [-lyp1, -lyp2, -lyp1, lyp1, lyp2, lyp1];
% A(2,:) = [lxp, 0, -lxp, -lxp, 0, lxp];
% A(3,:) = [kr, -kr, kr, -kr, kr, -kr];
% A(4,:) = [1, 1, 1, 1, 1, 1];
% A(5,:) = [0, 1, 0, 0, -1, 0];
% A(6,:) = [0.25, 0.5, 0.25, 0.25, 0.5, 0.25];
% B = [Mx; My; Mz; R; 0; 0];

x = linsolve(A, B)
% T2 = T5 = mean(T1 + T3 + T4 + T6)