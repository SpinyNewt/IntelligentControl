function [sys,x0,str,ts,simStateCompliance] = transfer_angle(t,x,u,flag)
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



% 初始化模块
function [sys,x0,str,ts,simStateCompliance] = mdlInitializeSizes
sizes = simsizes;
sizes.NumContStates = 0; 
sizes.NumDiscStates  = 2;
sizes.NumOutputs = 2;    % q1,q2
sizes.NumInputs = 2;   % q1,q2
sizes.DirFeedthrough = 1;  % must
sizes.NumSampleTimes = 1;
sys = simsizes(sizes);
x0 = [0 0];
str = [];
ts = [-1 0];
simStateCompliance = 'UnknownSimState';

function sys=mdlUpdate(t,x,u)
delta1 = get_angle(u(1),x(1));
delta2 = get_angle(u(2),x(2));
sys = [delta1,delta2];                    

function sys = mdlOutputs(t,x,u)
delta1 = get_angle(u(1),x(1));
delta2 = get_angle(u(2),x(2));
sys = [delta1,delta2]; 


function sys=mdlGetTimeOfNextVarHit(t,x,u)
sampleTime = 1;               
sys = t + sampleTime;         
 
function sys=mdlTerminate(t,x,u)
sys = [];


function [delta] = get_angle(u,delta)
q = u;
while q+delta>pi
    delta = delta - pi;
end
while q+ delta<-pi
    delta = delta + pi;
end