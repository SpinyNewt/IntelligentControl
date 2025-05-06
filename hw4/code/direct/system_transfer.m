function [sys,x0,str,ts,simStateCompliance] = system_transfer(t,x,u,flag)
switch flag,
  case 0,
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes;
  case 1,
    sys=mdlDerivatives(t,x,u);
  case 2,
    sys=mdlUpdate(t,x,u);
  case 3,
    sys=mdlOutputs(t,x,u);
  case 4,
    sys=mdlGetTimeOfNextVarHit(t,x,u);
  case 9,
    sys=mdlTerminate(t,x,u);
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));
end

function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes

sizes = simsizes;
sizes.NumContStates  = 4;   % q1,q2,dq1/dt,dq2/dt
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 2;   % q1,q2
sizes.NumInputs      = 2;   %  tau1,tau2
sizes.DirFeedthrough = 0;   
sizes.NumSampleTimes = 1; 
sys = simsizes(sizes);
x0  = [0 0 0 0];
str = [];
ts  = [-1 0];
simStateCompliance = 'UnknownSimState';

function sys=mdlDerivatives(t,x,u)
% Parameters
h1 = 0.0308;
h2 = 0.0106;
h3 = 0.0095;
h4 = 0.2086;
h5 = 0.0631;
g = 9.8;
% State variables and inputs
q1 = x(1); q2 = x(2) ; dq1 = x(3); dq2 = x(4);tau = u;
% Dynamics equations
m11 = h1 + h2 + 2*h3*cos(q2);
m12 = h2 + h3*cos(q2);
m21 = h2 + h3*cos(q2);
m22 = h2;
M = [m11 m12;
     m21 m22];
% Coriolis and gravity terms
c11 = -h3* sin(q2)*dq2;
c12 = -h3* sin(q2)*(dq1+dq2);
c21 = h3*sin(q2)*dq1;
c22 = 0;
C = [c11 c12;
     c21 c22];

g1 = h4*g*cos(q1) +h5*g*cos(q1+q2);
g2 = h5*g*cos(q1+q2);
G = [g1;g2];
% Acceleration equations
ddq = M\(tau-C*[dq1;dq2]-G);
ddq1 = ddq(1);
ddq2 = ddq(2);

sys = [dq1;dq2;ddq1;ddq2];


function sys=mdlUpdate(t,x,u)
sys = [];

function sys=mdlOutputs(t,x,u)
sys = [x(1) x(2)];


function sys=mdlGetTimeOfNextVarHit(t,x,u)

sampleTime = 1;    %  Example, set the next hit to be one second later.
sys = t + sampleTime;


function sys=mdlTerminate(t,x,u)

sys = [];

% end mdlTerminate