% This function parametrizes the grey box linear discrete time model and is
% called by idgrey command in driver_4_tank
function [A, B, C, D, K] = myDiscreteGreyBoxModel_Observer(par, Ts,aux)
% myDiscreteModel: Returns discrete-time state-space matrices
%   par(1): unknown pole location parameter
%   Ts: sample time
%   aux: auxiliary arguments (not used in this simple example)
    alpha = par(1:6);
    beta = par(7:12);
    gama = par(13:end) ;

    % Define matrices based on parameters (example of a simple 2nd order system)
    A = [alpha(1), 0, alpha(2), 0; 0, alpha(3), 0, alpha(4); 0 0 alpha(5) 0; 0 0 0 alpha(6)];
    B = [beta(1) beta(2); beta(3) beta(4); 0 beta(5); beta(6) 0];
    C = [0.5 0 0 0; 0 0.5 0 0];
    D = zeros(2);
    K = reshape(gama, [4, 2]); 
    
    % Note: since fcn_type is 'd', Ts is provided to the function, 
    % but in this simple example, we assume A and B are already in discrete form
    % and not dependent on the sampling time T (par(1) might be a Z-domain pole).
end

