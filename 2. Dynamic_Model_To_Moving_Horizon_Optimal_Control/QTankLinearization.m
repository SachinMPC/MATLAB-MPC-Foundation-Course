close all    % close all open figures 
clear all     % clear work space 
clc             % clear screen 

global QTank  % Define global variables 

QTank.samp_T = 4 ;             % sampling time 

% Four Tank system: Initialize model parameters

QTank.a1 = 0.071 ;    % cm^2 (Area of outlet tube for Tank 1)
QTank.A1 = 28    ;    % cm^2 (Area of tank 1)

QTank.a3 = 0.071 ;    % cm^2 (Area of outlet tube for Tank 3)
QTank.A3 = 28    ;    % cm^2 (Area of tank 3)

QTank.a2 = 0.057 ;    % cm^2 (Area of outlet tube for Tank 2)
QTank.A2 = 32    ;    % cm^2 (Area of tank 2)

QTank.a4 = 0.057 ;    % cm^2 (Area of outlet tube for Tank 4)
QTank.A4 = 32    ;    % cm^2 (Area of tank 4)

fprintf('\n  Quadruple Tank System: Linearized Model Development \n') ;
operating_pt = input('\n Specify Operating Condition. Type (1) for Minimum Phase and (2) for Non-minimum Phase : '); 

% Model parameters at minimum phase (P-) and non-minimum phase (P+) operating points

if ( operating_pt == 1 )                                  % Minimum phase system
    QTank.gam1 = 0.7 ;    % cm^3/Vs
    QTank.k1 =   3.33 ;    
    QTank.gam2 = 0.6 ;    % cm^3/Vs
    QTank.k2 =   3.35 ;
    QTank.Us = [ 3 3 ]' ;           % Steady state input 
elseif ( operating_pt == 2 )                           % Non-Minimum phase system 
    QTank.gam1 = 0.43 ;    % cm^3/Vs
    QTank.k1 =   3.14 ;
    QTank.gam2 = 0.34 ;    % cm^3/Vs
    QTank.k2 =   3.29 ;
    QTank.Us = [ 3.15 3.15 ]' ;   % Steady state input 
end 
QTank.Ds = 4;    % Steady state disturbance flow input 
QTank.gam3 = 0.4 ;   % For disturbance input 
    
% Find Equilibrium/ Steady State Operating Point for the specified inputs 

if ( operating_pt == 1 )                           % Minimum phase system              
   Xs0 = [ 12.4 12.7 1.8 1.4 ]' ;    % Initial Guess 
elseif ( operating_pt == 2 )                    % Non-Minimum phase system  
   Xs0 = [ 12.6 13 4.8 4.9 ]' ;       % Initial Guess
end  

oldopts = optimset ;
tolerance = 1e-10 ;
options = optimset(oldopts,'MaxIter', 1e6, 'Display','off', 'TolFun',tolerance, 'TolX', tolerance) ;
QTank.Xs = fsolve('QTank_SteadyState', Xs0, options ) ;

fprintf('\n Steady State corresponding to the specified input is as follows: ') ;
display(QTank.Xs')
fprintf('\n Hit ANY Key to Continue... \n') ;pause

QTank.C_mat = [ 0.5 0 0 0 
                           0 0.5 0 0 ] ;    % Measurement matrix 
QTank.Ys = QTank.C_mat * QTank.Xs ;

n_st = length( QTank.Xs ) ;   % No. of states 
n_op =  length( QTank.Ys ) ;   % No. of measured outputs  
n_ip = length( QTank.Us ) ;   % No. of manipulated inputs  
n_ud = length( QTank.Ds ) ;  % No of unmeasured disturbances 

% Generate linear perturbation model at the current steady state 
Z_vec = [ QTank.Xs' QTank.Us' QTank.Ds ]' ;
Jacob_mat = Num_Jacobian( 'QTank_JacobFn', Z_vec ) ;
% Continuous time linear perturbation model 
QTank.A_mat = Jacob_mat(:,1:n_st) ; 1

QTank.B_mat  = Jacob_mat(:,n_st+1:n_st+n_ip+n_ud) ;
QTank.D_mat = zeros(n_op,n_ip+n_ud) ; 
% Create a state space object in Matlab 
QTank.cmod = ss( QTank.A_mat, QTank.B_mat, QTank.C_mat, QTank.D_mat ) ;     

fprintf('\n Continuous Time Linear State Space Model at chosen equilibrium point') ;
QTank.cmod
fprintf('\n Hit ANY Key to Continue... \n') ;pause

fprintf('\n Laplace Transfer Function Matrix at chosen equilibrium point') ;
stf = tf( QTank.cmod )   
fprintf('\n Hit ANY Key to Continue... \n') ;pause

%  Generate discrete time linear perturbation model 

dmod = c2d( QTank.cmod, QTank.samp_T ) ;          % Continuous to discrete conversion 

fprintf('\n Discrete Time Linear State Space Model at the chosen equilibrium point') ;
dmod
fprintf('\n Hit ANY Key to Continue... \n') ;pause

QTank.phy = dmod.a ; 
QTank.gama_u = dmod.b(:,1:n_ip)  ;  
QTank.gama_d = dmod.b(:,n_ip+1:n_ip+n_ud) ;

fprintf('\n\t z-Transfer Function Matrix at chosen equilibrium point \t') ;
ztf = tf( dmod )   
fprintf('\n\t Hit ANY Key to Continue... \n') ;pause

% Standard deviations of measurement noise 
QTank.Q_mat =  diag([ 0.05^2 ])' ;   
% Standard deviations of Unmeasured white noise disturbance in inputs 
QTank.R_mat =  diag([ 0.05^2 0.05^2 ]') ;     

% if ( operating_pt == 1 )                           % Minimum phase system
%     save QTank_MinPhase_Model QTank
% elseif ( operating_pt == 2 )                    % Non-Minimum phase system 
%      save QTank_NonMinPhase_Model QTank 
% end 

