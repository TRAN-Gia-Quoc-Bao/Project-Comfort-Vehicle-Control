function [K1, K2, K3, K4, lambda, Q] = LQRLMI_cvx(A1, A2, A3, A4, B, Cz, Dz, x0) 
[nx, nx] = size(A1);
[nx, m] = size(B);
[pz, nx] = size(Cz);
[pz, m] = size(Dz);
cvx_begin sdp
    variable Q(nx, nx) semidefinite;
    variable Y1(m, pz);
    variable Y2(m, pz);
    variable Y3(m, pz);
    variable Y4(m, pz);
    variable lambda semidefinite;
    minimize(lambda)
    [lambda x0'; x0 Q] > 0;
    [Q*A1'+A1*Q+Y1'*B'+B*Y1    Q*Cz'+Y1'*Dz';      Cz*Q+Dz*Y1      -eye(pz)] < 0;
    [Q*A2'+A2*Q+Y2'*B'+B*Y2    Q*Cz'+Y2'*Dz';      Cz*Q+Dz*Y2      -eye(pz)] < 0;
    [Q*A3'+A3*Q+Y3'*B'+B*Y3    Q*Cz'+Y3'*Dz';      Cz*Q+Dz*Y3      -eye(pz)] < 0;
    [Q*A4'+A4*Q+Y4'*B'+B*Y4    Q*Cz'+Y4'*Dz';      Cz*Q+Dz*Y4      -eye(pz)] < 0;
cvx_end
    K1 = (Y1*inv(Q));
    K2 = (Y2*inv(Q));
    K3 = (Y3*inv(Q));
    K4 = (Y4*inv(Q));