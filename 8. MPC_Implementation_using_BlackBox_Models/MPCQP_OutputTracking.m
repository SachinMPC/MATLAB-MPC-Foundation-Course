
% Function for MPC QP formulation
% <--------------MPC QP function---------------
% Inputs: Measurement and setpoint at instant k 
% Output: Manipulated input at instant k 

function [mpc, u_k ]  = MPCQP_OutputTracking(mpc, y_k, r_k, k) 

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

    % MPC QP controller implementation 

     if (isempty(mpc.deluL)==0 )
        if (k > 2 )
            % b_vec = mpc.F_vec+mpc.B_mat*uk(:,k-1);
            % Note before QP calculations, mpc.uk = uk(:,k-1)
            b_vec = mpc.F_vec+mpc.B_mat* mpc.uk;
        else
            b_vec = mpc.F_vec +mpc.B_mat*zeros(mpc.n_ip,1) ;
        end
    else
        b_vec = mpc.F_vec ;
     end

    if ( k == 1)
        Fk_vec = mpc.F_mat*mpc.xkhat(:,k) ;
    else
        % Fk_vec = mpc.F_mat*mpc.xkhat(:,k) - 2 * mpc.psi_mat' * mpc.WdelU_mat * mpc.psi_0_mat * uk(:,k-1) ;
        % Note before QP calculations, mpc.uk = uk(:,k-1)
        Fk_vec = mpc.F_mat*mpc.xkhat(:,k) - 2 * mpc.psi_mat' * mpc.WdelU_mat * mpc.psi_0_mat * mpc.uk ;
    end
    Fk_vec = Fk_vec + 2 * mpc.Su_mat' * mpc.WY_mat * mpc.Se_mat *  mpc.dkhat_f(:,k)...
             - 2 * mpc.WU_mat * mpc.Iu_mat * mpc.us_k(:,k)  - 2 * mpc.Su_mat' * mpc.WY_mat * mpc.Ir_mat * r_k ;

    [Uf_k,FVAL] = quadprog(mpc.H_mat, Fk_vec, mpc.A_mat, b_vec,[],[],[],[],[],options)  ;
    u_k = Uf_k(1:mpc.n_ip) ;  % Implement only optimal u(k) 

    % Internal MPC model update from k to k+1
     mpc.xkhat(:,k+1) = mpc.phy * mpc.xkhat(:,k) + mpc.gam * u_k + mpc.L_mat *  mpc.dkhat(:,k) ;
end 
