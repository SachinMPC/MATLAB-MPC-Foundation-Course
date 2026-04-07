
%  Function for MPC QP formulation: 
% <--------------MPC QP function---------------
% Inputs: Measurement and setpoint at instant k 
% Output: Manipulated input at instant k 

function [mpc, u_k ]  = MPCQP_StateSpace(mpc, y_k, r_k, k) 

  % Estimate model plant mismatch and filter it using unity gain filter 
    mpc.yhatk(:,k) = mpc.C_mat * mpc.xkhat(:,k) ; 
    mpc.dkhat(:,k) = y_k - mpc.yhatk(:,k) ;  
    mpc.dkhat_f(:,k+1) = mpc.phy_d *  mpc.dkhat_f(:,k) + (eye(mpc.n_op) - mpc.phy_d) *  mpc.dkhat(:,k) ; 

    % Target state computation using quadratic programming 
     
    options=optimoptions('quadprog', 'display', 'off');

    Fsk_vec = mpc.Fs_mat * ( r_k - mpc.Gain_e * mpc.dkhat_f(:,k))  ; 
    [us_k_QP, fsval] = quadprog(mpc.Hs_mat, Fsk_vec,mpc.As_mat, mpc.bs_vec, [],[],[],[],[], options )  ;
    mpc.us_k(:,k) = us_k_QP ; 
    mpc.xs_k(:,k) = inv(eye(mpc.n_st) - mpc.phy) * ( mpc.gam * mpc.us_k(:,k) + mpc.L_mat * mpc.dkhat_f(:,k))  ;
   
     Fk_vec = mpc.F_mat*mpc.xkhat(:,k) - 2 * mpc.WU_mat * mpc.Iu_mat * mpc.us_k(:,k)...
              - 2 * mpc.Su_mat' * mpc.WX_mat * (mpc.Ix_mat * mpc.xs_k(:,k) - mpc.Se_mat * mpc.dkhat_f(:,k))  ; 
  
    [Uf_k,FVAL] = quadprog(mpc.H_mat, Fk_vec, mpc.A_mat, mpc.F_vec,[],[],[],[],[],options)  ;
    u_k = Uf_k(1:mpc.n_ip) ;  % Implement only optimal u(k) 

    % Internal MPC model: update from k to k+1
     mpc.xkhat(:,k+1) = mpc.phy * mpc.xkhat(:,k) + mpc.gam * u_k + mpc.L_mat *  mpc.dkhat(:,k) ;

end   %  End of MPC QP function 
