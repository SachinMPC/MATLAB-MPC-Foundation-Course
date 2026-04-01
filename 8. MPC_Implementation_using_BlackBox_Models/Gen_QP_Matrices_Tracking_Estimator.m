% This function generated QP MPC matrices for a special case when 
% the prediction and control horizon are equal 

function [mpc] = Gen_QP_Matrices_Tracking_Estimator(mpc)

phy = mpc.phy ; gam = mpc.gam ; L_mat = mpc.L_mat ; C_mat =  mpc.C_mat ;
Wy = mpc.Wy ;  Wu = mpc.Wu ;  n_pred= mpc.n_pred; u_L = mpc.u_L ; u_H = mpc.u_H ; deluL = mpc.deluL ; deluH = mpc.deluH ; 

% Generate Sx and Su matrices 

[n_st,n_ip] = size(gam) ;
[n_op,nx] = size(C_mat) ;

mat = zeros(n_st, n_ip * n_pred);
Sx_mat = [] ;     Su_mat = [] ;  Se_mat = [] ; 
WX_mat = [] ;     WU_mat = [] ; 
Se_row_mat = zeros(n_st, n_op) ;
c2 = 0 ; r2 = 0 ; 

% Find W infinity for open loop stable systems 

Wx = C_mat' * Wy * C_mat ; 
spectral_radius = max(abs(eig(phy))) ;
if (spectral_radius < 1)
    Wx_inf = dlyap(phy',Wx ) ;
end

for k = 1 : n_pred
    mat = phy*mat;  % Block row matrix for intermediate computations 
    
    % Generation of Su_mat   
    c1 = c2 ; c2 = c1 + n_ip ;
    mat(:,c1+1:c2) = gam;    
    Su_mat = [ Su_mat ; mat ] ;
    
    % Generation of Sx_mat
    Sx_mat = [ Sx_mat ;  phy^k ] ;

    % Generate state and input weighting matrices 
    
    WU_mat = blkdiag(WU_mat, Wu)  ;

    if (k == n_pred )
        if (spectral_radius < 1)
            WX_mat = blkdiag(WX_mat, Wx_inf)  ;
        else
            WX_mat = blkdiag(WX_mat, Wx)  ;
        end
    else
        WX_mat = blkdiag(WX_mat, Wx)  ;
    end

    % Generation of Se matrix 
     r1 = r2 ; r2 = r1 + n_st ;
    Se_row_mat = phy * Se_row_mat +  L_mat  ; 
    Se_mat(r1+1:r2,:) = Se_row_mat;    
end

% Generate QP matrices
H_mat = 2 * (Su_mat'* WX_mat * Su_mat + WU_mat ) ;
F_mat = 2 * Su_mat' * WX_mat * Sx_mat ; 

H_mat=(H_mat+H_mat')/2;  % To avoid warnings in quadprog 

% Matrices for Inclusion of U constraints

Iu_mat = [];       % Generation of I_u and I_x matrices
Ix_mat = [] ;
for i = 1:n_pred
    Ix_mat = [Ix_mat; eye(n_st)];
    Iu_mat = [Iu_mat; eye(n_ip)];
end
A_mat  = [eye(n_pred*n_ip);-eye(n_pred*n_ip)];
F_vec = [ Iu_mat* u_H; -Iu_mat* u_L ] ;
B_mat = zeros( 2*n_pred*n_ip, n_ip) ;

 % Inclusion of delta_U constraints

 if ( isempty(deluL)==0)
     % Generation of psi matrix
     psi_mat = diag(ones( n_ip* n_pred, 1)) - diag( ones(n_ip*(n_pred-1), 1),-n_ip)  ;

     % Generation of psi_0 matrix
     psi_0_mat = zeros( n_ip*n_pred,n_ip ) ;
     psi_0_mat(1:n_ip,1:n_ip) = eye(n_ip) ;

     A_mat = [ A_mat ; psi_mat ; - psi_mat ] ;
     F_vec = [F_vec ; Iu_mat*deluH ; -Iu_mat*deluL ] ;
     B_mat = [ B_mat ; psi_0_mat ; -psi_0_mat ] ;
 end

 % Populate mpc structure 
 mpc.Sx_mat = Sx_mat ;  mpc.Su_mat = Su_mat ;  mpc.Se_mat = Se_mat ;  
 mpc.H_mat = H_mat ;  mpc. F_mat = F_mat ;  mpc.A_mat= A_mat ;  
 mpc.F_vec = F_vec ; mpc. B_mat = B_mat;  
 mpc.WX_mat = WX_mat ;  mpc.WU_mat = WU_mat ;  mpc.Ix_mat = Ix_mat ;  mpc.Iu_mat = Iu_mat ; 

 % QP matrices for target state computation 

 mpc.Gain_ss = mpc.C_mat * inv(eye(mpc.n_st) - mpc.phy) * mpc.gam ;
mpc.Gain_e = eye(n_op) + mpc.C_mat * inv(eye(n_st)-mpc.phy)* mpc.L_mat ;

mpc.Hs_mat = 2 * mpc.Gain_ss' * mpc.Wy * mpc.Gain_ss ;
mpc.s_mat=(mpc.Hs_mat+mpc.Hs_mat')/2;
mpc.Fs_mat =  -2 * mpc.Gain_ss' * mpc.Wy  ; 
mpc.As_mat = [ eye(n_ip) ; -eye(n_ip)]  ;
mpc.bs_vec = [ mpc.u_H ; -mpc.u_L ]; 
mpc.Hs_mat=(mpc.Hs_mat+mpc.Hs_mat')/2;