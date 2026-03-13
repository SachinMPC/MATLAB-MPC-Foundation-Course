% QTank_JacobFn.M
% Quadruple Tank System  
% Ref.:  Johanson, K. H. IEEE Trans. On Control Systems Technology,
% Vol. 8, No. 3, May 2000
% Function called by Jacobian calculation by numerical differentiation 

function xdot = QTank_JacobFn( Z )

global QTank 

gc = 981 ;  % (cm/s^2) gravitational constnat 

% Assign elements of Z to appropriate variables 

h(1) = Z(1) ; h(2) = Z(2) ; h(3) = Z(3) ; h(4) = Z(4) ;

U_k(1) = Z(5) ; U_k(2) = Z(6) ; D_k = Z(7) ;

hdot(1) = -(QTank.a1/QTank.A1) * (2 * gc * h(1))^0.5 ;
hdot(1) = hdot(1) + (QTank.a3/QTank.A1) * (2 * gc * h(3))^0.5 ;
hdot(1) = hdot(1) + (QTank.gam1*QTank.k1/QTank.A1) * U_k(1) ;

hdot(2) = -(QTank.a2/QTank.A2) * (2 * gc * h(2))^0.5 ;
hdot(2) = hdot(2) + (QTank.a4/QTank.A2) * (2 * gc * h(4))^0.5 ;
hdot(2) = hdot(2) + (QTank.gam2*QTank.k2/QTank.A2) * U_k(2) ;

hdot(3) = -(QTank.a3/QTank.A3) * (2 * gc * h(3))^0.5 ;
hdot(3) = hdot(3) + ((1-QTank.gam2)*QTank.k2/QTank.A3) * U_k(2) ;
hdot(3) = hdot(3) + (QTank.gam3/ QTank.A3) * D_k  ;

hdot(4) = -(QTank.a4/QTank.A4) * (2 * gc * h(4))^0.5 ;
hdot(4) = hdot(4) + ((1-QTank.gam1)*QTank.k1/QTank.A4) * U_k(1) ;
hdot(4) = hdot(4) + ((1-QTank.gam3)/ QTank.A4) * D_k  ;

xdot = hdot' ;