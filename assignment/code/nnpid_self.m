function [sys, x0, str, ts] = nnpid_self(t, x, u, flag)
    % Neural Network PID Control S-Function
    % Input: u = [x_0, y_0, x_1, y_1, x_2, y_2, bx_0, by_0, bx_1, by_1]
    % Output: [du_x, du_y]

    switch flag
        case 0 % initilize
            [sys, x0, str, ts] = mdlInitializeSizes;
        case 2 % calculate deritive
            sys = mdlUpdate(t, x, u);
        case 3 % calculate output
            sys = mdlOutputs(t, x, u);
        otherwise % other circumstances
            sys = [];
    end
end

%% Initialization
function [sys, x0, str, ts] = mdlInitializeSizes
    global wi wo xite alfa wo_prev wi_prev u1_1;
    u1_1 = [0;0];
    rng('default'); % For reproducibility
    wi = randn(10, 8);  % Input to hidden layer weights (10x8)
    wo = randn(6, 10);  % Hidden to output layer weights (6x10)
    xite = 0.05;  % Learning rate
    alfa =0.01;  % Momentum term
    wo_prev = wo;  % Previous output layer weights
    wi_prev = wi;  % Previous hidden layer weights
    % Initialize the sizes structure
    sizes = simsizes;
    sizes.NumContStates  = 0;  % No continuous states
    sizes.NumDiscStates = numel(wi) + numel(wo);  % Total number of weights
    sizes.NumOutputs     = 2;  % Outputs: [du_x, du_y]
    sizes.NumInputs      = 6; % Inputs: [e_x, e_y, e_x_1, e_y_1, e_x_2,e_y_2]
    sizes.DirFeedthrough = 1;  % Direct feedthrough (output depends on input)
    sizes.NumSampleTimes = 1;  % Single sample time
    sys = simsizes(sizes);

    % Initialize neural network weights
    x0 = [wi(:); wo(:)];  % Initial weights
    str = [];
    ts  = [0.001 0];  % Sample time: [period, offset]
end

%% Update discrete states (neural network weights)
function sys = mdlUpdate(~, x, u)
    global wi wo xite alfa wo_prev wi_prev;
    wi = reshape(x(1:80), [10, 8]);  % First 80 elements are wi
    wo = reshape(x(81:140), [6, 10]);  % Next 60 elements are wo

    % Inputs
    p_x = u(1);p_y=u(2); i_x=u(3); i_y=u(4);d_x=u(5);d_y=u(6);

    % Neural network input
    xi = [p_x; i_x ; d_x; 1; p_y;i_y;d_y; 1];  % Include bias term
    % Hidden layer output
    I = wi * xi;  % Hidden layer input
    Oh = max(0, I);  % Hidden layer activation (ReLU)

    % Output layer output
    K = wo * Oh;  % Output layer input
    K = 1 ./ (1 + exp(-K));  % Output layer activation (sigmoid)

    % Error for weight update
    error = [ p_x;i_x; d_x;p_y;i_y;d_y];

    % Output layer gradient
    dK = K .* (1 - K);  % Sigmoid derivative
    delta3 = dK .* error;  % Element-wise product

    % Hidden layer gradient
    dOh = (I > 0);  % ReLU derivative
    delta2 = (wo' * delta3) .* dOh;  % Backpropagate error

    % Weight update
    d_wo = xite * (delta3 * Oh') + alfa * (wo - wo_prev);
    d_wi = xite * (delta2 * xi') + alfa * (wi - wi_prev);

    % Update weights
    wo_prev = wo;
    wi_prev = wi;
    wo = wo - d_wo;
    wi = wi - d_wi;

    % Update discrete states (weights)
    sys = [wi(:); wo(:)];  % Flatten weights into a single vector
end

%% Calculate outputs
function sys = mdlOutputs(~, ~, u)
    global wi wo u1_1;  % Neural network weights

    % Inputs
    p_x = u(1);p_y=u(2); i_x=u(3); i_y=u(4);d_x=u(5);d_y=u(6);

    % Neural network input
    xi = [p_x; i_x ; d_x; 1; p_y;i_y;d_y; 1];  % Include bias term
    % Hidden layer output
    I = wi * xi;  % Hidden layer input
    Oh = max(0, I);  % Hidden layer activation (ReLU)

    % Output layer output
    K = wo * Oh;  % Output layer input
    K = 1 ./ (1 + exp(-K));  % Output layer activation (sigmoid)

    % PID parameters
    kp_x = K(1)+1; ki_x = K(2)+1; kd_x = K(3)+1;
    kp_y = K(4)+1; ki_y = K(5)+1; kd_y = K(6)+1;

    % Control outputs  
    u_x = kp_x * p_x + 4*ki_x * i_x +2*kd_x * d_x;
    u_y = kp_y * p_y + 0.1*ki_y * i_y + 0.55*kd_y * d_y;

    % Output
    u1_1(1)=u_x;
    u1_1(2)= u_y;
    sys = u1_1;  % Output is 2-dimensional
end