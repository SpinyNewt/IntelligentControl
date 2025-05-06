% 普通PID控制器
function [sys,x0,str,ts] = pid_plain(t,x,u,flag)
switch flag, 
  case 0,
    [sys,x0,str,ts]=mdlInitializeSizes; 
  case 2,
    sys=mdlUpdate(t,x,u); 
  case 3,
    sys=mdlOutputs(t,x,u); 
  case 4,
    sys=mdlGetTimeOfNextVarHit(t,x,u);  
  case 9
    sys=mdlTerminate(t,x,u);
 
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));
 
end

% 初始化模块
function [sys,x0,str,ts,simStateCompliance] = mdlInitializeSizes
sizes = simsizes;
sizes.NumContStates = 0; 
sizes.NumDiscStates  = 3; % F(k), x(k-1),x(k)
sizes.NumOutputs = 1;    % output F(k)
sizes.NumInputs = 2;     % input x/50, θ, but only uses θ
sizes.NumSampleTimes = 1;
sizes.DirFeedthrough = 1;  % must
sys = simsizes(sizes);
x0 = [0; get_theta(); get_theta()];   
str = [];
ts=[0.0001 0]; % sample T = 0.0001
simStateCompliance = 'UnknownSimState';


% updating
function sys = mdlUpdate(t,x,u)
Kp=200;Ti=0.001;Td=10;T=0.0001;K=1;
if isnan(u(2))
    u(2) = get_theta(); % prevent input nan if system starts with pid module ranther than system transfer module
end
% pid controller
deltak = u(2) - x(3); 
deltak_1 = x(3) -x(2);
F = x(1) + K*(Kp*deltak+ T/Ti *u(2)+ Td/T *(deltak - deltak_1)); 
F_m = get_F();
% limit F to [-Fm,Fm]
if F>F_m
    F=F_m;
elseif F<-F_m
    F=-F_m;
end

sys = [F;x(3);u(2)];



function sys = mdlOutputs(t,x,u)
% the sfunction do outputing before updating, so x(1) is F(k-1), we need
% F(k)
Kp=200;Ti=0.001;Td=10;T=0.0001;K=1;
if isnan(u(2))
    u(2) = get_theta();
end
deltak = u(2) - x(3);
deltak_1 = x(3) -x(2);
F = x(1) + K*(Kp*deltak+ T/Ti *u(2)+ Td/T *(deltak - deltak_1));
F_m = get_F();
if F>F_m
    F=F_m;
elseif F<-F_m
    F=-F_m;
end
sys=F;




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