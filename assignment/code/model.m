function [sys,x0,str,ts,simStateCompliance] = model(t,x,u,flag)
    switch flag
        case 0 % Initialization
            [sys,x0,str,ts,simStateCompliance] = mdlInitializeSizes;
        case 1 % Derivatives
            sys = mdlDerivatives(t,x,u);
        case 3 % Outputs
            sys = mdlOutputs(t,x,u);
        otherwise
            sys = [];
    end
end

%% Initialization Function
function [sys,x0,str,ts,simStateCompliance] = mdlInitializeSizes
    sizes = simsizes;
    sizes.NumContStates  = 6; % 6 states: [X,u,Y,v,phi,r]
    sizes.NumDiscStates  = 0;
    sizes.NumOutputs     = 2; % Outputs: [X,Y]
    sizes.NumInputs      = 2; % Inputs:  [Fx,delta_f]
    sizes.DirFeedthrough = 1; % Direct feedthrough of inputs
    sizes.NumSampleTimes = 1;
    sys = simsizes(sizes); 
    x0  = [0,20,0,0,0,0];% Initial conditions for the states
    str = [];
    ts  = [-1 0]; % Continuous system
    simStateCompliance = 'UnknownSimState';
end

%% Derivatives Function (Quadrotor Dynamics)
function sys = mdlDerivatives(t,x,u)
% given parameters of system
Nw=2; f=0.01; Iz=2667; a=1.35; b=1.45; 
By=0.27; Cy=1.2; Dy=0.7; Ey=-1.6;
Shy=0; Svy=0; m=1400; g=9.806;
F_x = u(1);
delta_f = u(2);

%slip angle functions in degrees
a_f=rad2deg(delta_f-atan2(x(4)+a*x(6),x(2)));
a_r=rad2deg(-atan2((x(4)-b*x(6)),x(2)));
%Nonlinear Tire Dynamics
phi_yf=(1-Ey)*(a_f+Shy)+(Ey/By)*atan(By*(a_f+Shy));
phi_yr=(1-Ey)*(a_r+Shy)+(Ey/By)*atan(By*(a_r+Shy));

F_zf=b/(a+b)*m*g;
F_yf=F_zf*Dy*sin(Cy*atan(By*phi_yf))+Svy;

F_zr=a/(a+b)*m*g;
F_yr=F_zr*Dy*sin(Cy*atan(By*phi_yr))+Svy;

F_total=sqrt((Nw*F_x)^2+(F_yr^2));
F_max=0.7*m*g;
% saturation of input F
if abs(F_total)>F_max
    
    F_x=F_max/abs(F_total)*F_x;
  
    F_yr=F_max/abs(F_total)*F_yr;
end

%vehicle dynamics 
sys= [x(2)*cos(x(5))-x(4)*sin(x(5));...
          (-f*m*g+Nw*F_x-F_yf*sin(delta_f))/m+x(4)*x(6);...
          x(2)*sin(x(5))+x(4)*cos(x(5));...
          (F_yf*cos(delta_f)+F_yr)/m-x(2)*x(6);...
          x(6);...
          (F_yf*a*cos(delta_f)-F_yr*b)/Iz];
end


%% Outputs Function (Quadrotor States)
function sys = mdlOutputs(t,x,u)
    % Return the current states: x,y
    sys = [x(1);x(3)];
    % sys = x;
end
