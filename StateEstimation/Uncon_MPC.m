% This function generates matrices for reformulating MPC as a QP problem
% Returns a structure 'mpc' that contains these matrices
% Solver: quadprog (Matlab optimization toolbox) 
% Author: Sachin C. Patwardhan, Chemical Engineering, IIT Bombay

function G_mat  = Uncon_MPC( phy, gama, Wx, Wu, N_pred  )
global  n_st n_op n_ip 
% Intermediate matrices for calculations 
Sx_mat = [] ;     
Su_mat = [] ;
N_con = N_pred ; 

[n_st, n_ip] = size( gama ) ;

mat = zeros(n_st, n_ip*N_pred);
c2 = 0 ;
temp_mat = eye(size(phy)) ;
WX = [] ;  WU = [] ;
C_mat = eye(n_st) ;

for k = 1 : N_pred
    
    WX = blkdiag( WX, Wx) ;
    WU = blkdiag( WU, Wu) ;
    
    % Generation of Seta_mat
    if ( k < N_pred )
        temp_mat = phy*temp_mat + eye(size(phy)) ;
    end
    mat = phy*mat;
    
    % Generation of Su_mat   
    c1 = c2 ; c2 = c1 + n_ip ;
    mat(:,c1+1:c2) = gama;    
    Su_mat = [ Su_mat ; C_mat*mat ] ;
    
    % Generation of Sx_mat
    Sx_mat = [ Sx_mat ; C_mat*phy^k ] ;
end

H_mat = 2 * ( Su_mat' * WX * Su_mat + WU ) ;
F_mat = 2 * Su_mat' * WX * Sx_mat ;

% %  Generation of transformation mattrix for restricting the input move to
% %  control horizon
% psi_pq_mat = [];
% for i = 1:length(mpc.ip_block_vec)  % With Input Blocking 
%     I_mi = [] ;
%     for j = 1:mpc.ip_block_vec(i)
%         I_mi = abv( I_mi, eye(mpc.n_ip) ) ;
%     end
%     psi_pq_mat = daug( psi_pq_mat, I_mi );
% end
% 
% % Generation of Su_mat taking into account control horizon
% Su_mat = Su_mat * psi_pq_mat;

% % Generation of psi matrix
% psi_mat = diag(ones( mpc.n_ip*mpc.N_con, 1))-diag( ones( mpc.n_ip*(mpc.N_con-1), 1),-mpc.n_ip)  ;

% Generation of psi_0 matrix
psi_0_mat = zeros( n_ip*N_con,n_ip ) ;
psi_0_mat(1:n_ip,1:n_ip) = eye(n_ip) ;

G_mat =  psi_0_mat' * inv(H_mat) * F_mat ;

% % Generation of psi_u matrix
% psi_u = [];
% for i = 1:mpc.N_con
%     psi_u = [psi_u; eye(mpc.n_ip)];
% end

return