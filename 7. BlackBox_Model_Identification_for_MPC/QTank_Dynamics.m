% QTank_Dynamics.M
% Modified Quadruple Tank System  
% Ref.:  Johanson, K. H. IEEE Trans. On Control Systems Technology,
% Vol. 8, No. 3, May 2000
% Function called by ODE solver for Dynamic Simulation 
% Given time and X(t), this function computes f(X(t), U(k), D(k)) 

function xdot = QTank_Dynamics(time, Xt)

global QTank

gc = 981 ;  % (cm/s^2) gravitational constant 

h(1) = Xt(1) ; h(2) = Xt(2) ; h(3) = Xt(3) ; h(4) = Xt(4) ;

hdot(1) = -(QTank.a1 / QTank.A1) * (2 * gc * h(1))^0.5 ;
hdot(1) = hdot(1) + (QTank.a3 / QTank.A1) * (2 * gc * h(3))^0.5 ;
hdot(1) = hdot(1) + (QTank.gam1 * QTank.k1 / QTank.A1) * QTank.Uk(1) ;

hdot(2) = -(QTank.a2 / QTank.A2) * (2 * gc * h(2))^0.5 ;
hdot(2) = hdot(2) + (QTank.a4 / QTank.A2) * (2 * gc * h(4))^0.5 ;
hdot(2) = hdot(2) + (QTank.gam2 * QTank.k2 / QTank.A2) * QTank.Uk(2) ;

hdot(3) = -(QTank.a3 / QTank.A3) * (2 * gc * h(3))^0.5 ;
hdot(3) = hdot(3) + ((1-QTank.gam2)*QTank.k2 / QTank.A3) * QTank.Uk(2) ;
hdot(3) = hdot(3) + (QTank.gam3/ QTank.A3) * QTank.Dk  ;
;
hdot(4) = -(QTank.a4 / QTank.A4) * (2 * gc * h(4))^0.5 ;
hdot(4) = hdot(4) + ((1-QTank.gam1) * QTank.k1 / QTank.A4) * QTank.Uk(1) ;
hdot(4) = hdot(4) + ((1-QTank.gam3)/ QTank.A4) * QTank.Dk  ;

xdot = hdot' ;
