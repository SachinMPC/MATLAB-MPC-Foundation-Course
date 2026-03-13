% This function generated QP MPC matrices for a special case when 
% the prediction and control horizon are equal 

function [Sx_mat, Su_mat, H_mat, F_mat, A_mat, F_vec, B_mat] =...
                                                   Gen_QP_Matrices(phy, gam, Wx, Wu, n_pred, u_L, u_H, deluL, deluH)

% Generate Sx and Su matrices 

[n_st,n_ip] = size(gam) ;

mat = zeros(n_st, n_ip * n_pred);
Sx_mat = [] ;     Su_mat = [] ;
WX_mat = [] ;     WU_mat = [] ; 
c2 = 0 ;

for k = 1 : n_pred
    mat = phy*mat;  % Block row matrix for intermediate computations 
    
    % Generation of Su_mat   
    c1 = c2 ; c2 = c1 + n_ip ;
    mat(:,c1+1:c2) = gam;    
    Su_mat = [ Su_mat ; mat ] ;
    
    % Generation of Sx_mat
    Sx_mat = [ Sx_mat ; phy^k ] ;

    % Generate state and input weighting matrices 
    WX_mat = blkdiag(WX_mat, Wx)  ;
    WU_mat = blkdiag(WU_mat, Wu)  ;   
end

% Generate QP matrices
H_mat = 2 * (Su_mat'* WX_mat * Su_mat + WU_mat ) ;
F_mat = 2 * Su_mat' * WX_mat * Sx_mat ; 

% Matrices for Inclusion of U constraints

psi_u = [];       % Generation of psi_u matrix
for i = 1:n_pred
    psi_u = [psi_u; eye(n_ip)];
end
A_mat  = [eye(n_pred*n_ip);-eye(n_pred*n_ip)];
F_vec = [ psi_u* u_H; -psi_u* u_L ] ;
B_mat = zeros( 2*n_pred*n_ip, n_ip) ;

 % Inclusion of delta_U constraints

 if ( isempty(deluL)==0)
     % Generation of psi matrix
     psi_mat = diag(ones( n_ip* n_pred, 1)) - diag( ones(n_ip*(n_pred-1), 1),-n_ip)  ;

     % Generation of psi_0 matrix
     psi_0_mat = zeros( n_ip*n_pred,n_ip ) ;
     psi_0_mat(1:n_ip,1:n_ip) = eye(n_ip) ;

     A_mat = [ A_mat ; psi_mat ; - psi_mat ] ;
     F_vec = [F_vec ; psi_u*deluH ; -psi_u*deluL ] ;
     B_mat = [ B_mat ; psi_0_mat ; -psi_0_mat ] ;
 end
