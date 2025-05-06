% Nothing changed, skip if you want to save time: This function defines the subfunctions of the s-function
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



% initialization
function [sys,x0,str,ts,simStateCompliance] = mdlInitializeSizes
sizes = simsizes;
sizes.NumContStates = 3; % x, dx/dt, I
sizes.NumDiscStates  = 0;
sizes.NumOutputs = 1;    % x
sizes.NumInputs = 1;     
sizes.NumSampleTimes = 1;
sys = simsizes(sizes);
x0 = [0.03;0;0 ];    % initial value：x = 0.03, dx/dt = 0, I = 0
str = [];
ts = [-1 0];
simStateCompliance = 'UnknownSimState';


function sys = mdlDerivatives(t,x,u)
[m,g,K,R,L] = get_system_param();
X = x(1); X_dot = x(2); I = x(3); U = u(1);
dx1 = X_dot;
dx2 = (K*I^2/X^2 -m*g)/m;
dx3 = 1/L * (U - K*I*X_dot/X -R*I);
sys = [dx1; dx2; dx3];

function sys=mdlUpdate(t,x,u)
sys = [];                    

% output x
function sys = mdlOutputs(t,x,u)
sys = [x(1)]; 


function sys=mdlGetTimeOfNextVarHit(t,x,u)
sampleTime = 1;               
sys = t + sampleTime;         
 
function sys=mdlTerminate(t,x,u)
sys = [];