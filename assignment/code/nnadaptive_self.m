function [sys,x0,str,ts,simStateCompliance] = nnadaptive_self(t,x,u,flag)
    switch flag
        case 0, % initialization
            [sys,x0,str,ts,simStateCompliance] = mdlInitializeSizes;
        case 1, % calculate deritive
            sys = mdlDerivatives(t,x,u);
        case 3, % calculate output
            sys = mdlOutputs(t,x,u);
        otherwise % other circumstances
            sys = [];
    end
end

% initialization
function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes
    sizes = simsizes;
    sizes.NumContStates = 10; % neural network hidden size
    sizes.NumDiscStates = 0;
    sizes.NumOutputs = 2; % ouput[F_x,delta_f]
    sizes.NumInputs = 8; % input [Xref,Yref,dXref,dYref,X,Y,dX,dY]
    sizes.DirFeedthrough = 1;
    sizes.NumSampleTimes = 1;
    sys = simsizes(sizes);

    % initialize params
    global b c c_X c_Y d
    x0 = 0.1*ones(1,10); % initial weights
    c = 0.5*randn(4,10); % randomly initialize centers
    b = 10.0; % innitialize b
    c_X = 1; % Synovial control param
    c_Y = 0.5;
    d = 0.00001;% initial control param

    str = [];
    ts = [-1 0];
    simStateCompliance = 'UnknownSimState';
end

% deritive
function sys=mdlDerivatives(t,x,u)
    global b c c_X c_Y d
    % goal
    X_g = u(1); 
    Y_g = u(2); 
    dX_g = u(3); 
    dY_g = u(4); 

    % raal state
    X = u(5);
    Y = u(6);
    dX = u(7);
    dY = u(8);

    % calculate difference
    e_X = X - X_g;
    e_Y = Y - Y_g;
    de_X = dX - dX_g;
    de_Y = dY - dY_g;

    % Synovial control variable
    s_X = c_X*e_X + de_X;
    s_Y = c_Y*e_Y + 1*de_Y;

    W = x(1:10); % initial weights of NN network
    xi = [X; Y; dX; dY]; % state vector

    % RBF output
    h = zeros(10,1);
    for j = 1:10
        h(j) = d * exp(-norm(xi-c(:,j))^2/(2*b^2));
    end

    % adaptive control
    gama = 1000;
    for i = 1:10
        sys(i) = gama*(s_X*h(i) + s_Y*h(i)); % update weights
    end
end

% output
function sys=mdlOutputs(t,x,u)
    global b c c_X c_Y d
    % goal
    X_g = u(1); 
    Y_g = u(2);
    dX_g = u(3); 
    dY_g = u(4); 

    % real state
    X = u(5);
    Y = u(6);
    dX = u(7);
    dY = u(8);

    % calculate error
    e_X = X - X_g;
    e_Y = Y - Y_g;
    de_X = dX - dX_g;
    de_Y = dY - dY_g;

    % Synovial control variable
    s_X = c_X*e_X + de_X;
    s_Y = c_Y*e_Y + de_Y;

    % RBF output
    W = x(1:10); % NN weights
    xi = [X; Y; dX; dY]; % state vector
    h = zeros(10,1);
    for j = 1:10
        h(j) = d * exp(-norm(xi-c(:,j))^2/(2*b^2));
    end

    % adaptive control
    fn = W'*h; % output of NN network
    eta_X = 2.0; % stability factor
    eta_Y = 0.1;
    ut_X = -s_X - fn - eta_X * sign(s_X); % ux
    ut_Y = -s_Y - fn - eta_Y * sign(s_X) ; % uy
    
    % output
    sys =[1000*ut_X; 0.6*ut_Y]; 
    end
