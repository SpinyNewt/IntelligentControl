% set system transfer s-function
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
sizes.NumContStates = 4; % x, dx/dt, θ, dθ/dt
sizes.NumDiscStates  = 0;
sizes.NumOutputs = 2;    % output x, θ
sizes.NumInputs = 1;     
sizes.NumSampleTimes = 1;
sys = simsizes(sizes);
x0 = [0; 0; get_theta(); 0];    % initial value：x = 0, dx/dt = 0, θ = get_theta(),π/4 for default, dθ/dt = 0
str = [];
ts = [-1 0];
simStateCompliance = 'UnknownSimState';


% calculate derivatives
function sys = mdlDerivatives(t,x,u)
% set system params
M = 1; m = 0.5; l = 0.5; g = 9.8;
theta = x(3);
theta_dot = x(4);
F = u(1); % input F
% transfer functions
dx1 = x(2); % dx/dt
dx2 = (F + m * l * theta_dot^2 * sin(theta) - m * g * sin(theta)*cos(theta)) / (M + m - m * cos(theta)^2); % d^2x/dt^2 
dx3 = x(4); % dθ/dt
dx4 = ( (M+m)*g*(sin(theta)) -cos(theta)*(F+m*l*sin(theta)*theta_dot^2) )/((M+m)*l-m*l*cos(theta)^2); % d^2θ/dt^2
sys = [dx1; dx2; dx3; dx4];

function sys=mdlUpdate(t,x,u)
sys = [];                     %sys updates the system to the next state

function sys = mdlOutputs(t,x,u)
sys = [x(1)/50;x(3)];  % output x/50 (for display conveniece), 0


function sys=mdlGetTimeOfNextVarHit(t,x,u)
%计算下一个采样时间
%仅在系统是变采样时间系统时调用
sampleTime = 1;               %设置下一次采样时间是在1s以后
sys = t + sampleTime;         %sys表示下一个采样时间点
%% 
function sys=mdlTerminate(t,x,u)
%仿真结束时要调用的回调函数
%在仿真结束时，可以在此完成仿真结束所需的必要工作
sys = [];