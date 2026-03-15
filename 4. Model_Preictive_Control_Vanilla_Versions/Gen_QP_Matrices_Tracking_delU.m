% This function generated QP MPC matrices for a special case when 
% the prediction and control horizon are equal 
% function [Sx_mat, Su_mat, H_mat, F_mat, A_mat, F_vec, B_mat, WX_mat, WU_mat, WdelU_mat, Ix_mat, Iu_mat, psi_mat, psi_0_mat] =...
%                                     Gen_QP_Matrices_Tracking_delU(phy, gam, Wx, Wu, Wdelu, n_pred, u_L, u_H, deluL, deluH)

function [mpc] = Gen_QP_Matrices_Tracking_delU( mpc )

% Create local matrices from mpc structure 

phy = mpc.phy ; gam = mpc.gam ; Wx = mpc.Wx ;  Wu = mpc.Wu ;  Wdelu = mpc.Wdelu ;
n_pred= mpc.n_pred; u_L = mpc.u_L ; u_H = mpc.u_H ; deluL = mpc.deluL ; deluH = mpc.deluH ; 

% Generate Sx and Su matrices 

[n_st,n_ip] = size(gam) ;

mat = zeros(n_st, n_ip * n_pred);
Sx_mat = [] ;     Su_mat = [] ;
WX_mat = [] ;     WU_mat = [] ; 
WdelU_mat = [] ;
c2 = 0 ;

% Find W infinity for open loop stable systems 

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
    Sx_mat = [ Sx_mat ; phy^k ] ;

    % Generate state and input weighting matrices 
    
    WU_mat = blkdiag(WU_mat, Wu)  ;
    WdelU_mat = blkdiag(WdelU_mat, Wdelu)  ;

    if (k == n_pred )
        if (spectral_radius < 1)
            WX_mat = blkdiag(WX_mat, Wx_inf)  ;
        else
            WX_mat = blkdiag(WX_mat, Wx)  ;
        end
    else
        WX_mat = blkdiag(WX_mat, Wx)  ;
    end

end

 % Generation of psi matrix
 psi_mat = diag(ones( n_ip* n_pred, 1)) - diag( ones(n_ip*(n_pred-1), 1),-n_ip)  ;

 % Generation of psi_0 matrix
 psi_0_mat = zeros( n_ip*n_pred,n_ip ) ;
 psi_0_mat(1:n_ip,1:n_ip) = eye(n_ip) ;

% Generate QP matrices
H_mat = 2 * (Su_mat'* WX_mat * Su_mat + WU_mat  +  psi_mat' * WdelU_mat *  psi_mat) ;
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

 % Inclusion of delta_U constraints and delta_U penalty 

 if ( isempty(deluL)==0)
     A_mat = [ A_mat ; psi_mat ; - psi_mat ] ;
     F_vec = [F_vec ; Iu_mat*deluH ; -Iu_mat*deluL ] ;
     B_mat = [ B_mat ; psi_0_mat ; -psi_0_mat ] ;
 end

  % Populate mpc structure 
 mpc.Sx_mat = Sx_mat ;  mpc.Su_mat = Su_mat ;  
 mpc.H_mat = H_mat ;  mpc. F_mat = F_mat ;  mpc.A_mat= A_mat ;  
 mpc.F_vec = F_vec ; mpc. B_mat = B_mat;  
 mpc.WX_mat = WX_mat ;  mpc.WU_mat = WU_mat ; mpc. WdelU_mat =  WdelU_mat ;
 mpc.Ix_mat = Ix_mat ;  mpc.Iu_mat = Iu_mat ;  
 mpc.psi_mat = psi_mat ;  mpc.psi_0_mat = psi_0_mat ; 

 % QP matrices for target state computation

 mpc.Gain_ss = mpc.C_mat * inv(eye(mpc.n_st) - mpc.phy) * mpc.gam ;

 mpc.Wys = eye(mpc.n_op);
 mpc.Hs_mat = 2 * mpc.Gain_ss' * mpc.Wys * mpc.Gain_ss ;
 mpc.Hs_mat=(mpc.Hs_mat+mpc.Hs_mat')/2;
 mpc.Fs_mat =  -2 * mpc.Gain_ss' * mpc.Wys  ;
 mpc.As_mat = [ eye(mpc.n_ip) ; -eye(mpc.n_ip)]  ;
 mpc.bs_vec = [ mpc.u_H ; -mpc.u_L ];