clear;
% Reference trajcetory [v,curvature]

% Parameters
N = 50;
dt = 0.1;
v_k = 5.5;
delta_r = 0.005;
mass_ = 683;
lf_ = 0.758;
lr_ = 1.036;
wheelbase_ = lf_ + lr_;
cf_ = 24000;
cr_ = 21000;
iz_ = 560.94;
curvature = lr_ * mass_ / (2 * cf_ * wheelbase_) - lf_ * mass_ / (2 * cr_ * wheelbase_);

% Kinematic bicycle model
A_k = eye(2) + [0 v_k;0 0]*dt;
B_k = [0; v_k/(wheelbase_*cos(delta_r)*cos(delta_r))]*dt;
W_k = [0; -v_k*delta_r/(wheelbase_*cos(delta_r)*cos(delta_r))]*dt;

A_k = A_k + W_k;
% Dynamic bicycle model
Ad = zeros(4,4);
Ad(1, 2) = 1.0;
Ad(2, 2) = -(cf_ + cr_) / (mass_ * v_k);
Ad(2, 3) = (cf_ + cr_) / mass_;
Ad(2, 4) = (lr_ * cr_ - lf_ * cf_) / (mass_ * v_k);
Ad(3, 4) = 1.0;
Ad(4, 2) = (lr_ * cr_ - lf_ * cf_) / (iz_ * v_k);
Ad(4, 3) = (lf_ * cf_ - lr_ * cr_) / iz_;
Ad(4, 4) = -(lf_ * lf_ * cf_ + lr_ * lr_ * cr_) / (iz_ * v_k);
% Bilinear discretization
Ad_inv = inv(eye(4) - dt * 0.5 * Ad);

Bd = zeros(4,1);
Bd(1, 1) = 0.0;
Bd(2, 1) = cf_ / mass_;
Bd(3, 1) = 0.0;
Bd(4, 1) = lf_ * cf_ / iz_;

Wd = zeros(4,1);
Wd(1, 1) = 0.0;
Wd(2, 1) = (lr_ * cr_ - lf_ * cf_) / (mass_ * v_k) - v_k;
Wd(3, 1) = 0.0;
Wd(4, 1) = -(lf_ * lf_ * cf_ + lr_ * lr_ * cr_) / (iz_ * v_k);

Ad = Ad_inv*(eye(4)*dt*0.5*Ad);
Bd = Ad_inv*dt*Bd;
Wd = Ad_inv*dt*curvature*v_k*Wd;
Ad = Ad + Wd;

% Model initialization - choose the required model - kinematic or dynamic.
A = A_k;
B = B_k;

% Weighting matrix
%Q   = 50*eye(size(A,2));
Q = [50 0; 0 1];
R   = 0.1;

% init state
x{1} = [0.1;0.05];

% LMIs initialization
constraints = [];
[ma,na] = size(A);
[mb,nb] = size(B);
G = sdpvar(size(A,2),size(A,2));
Y = sdpvar(size(B,2),size(B,1),'full');
gamma = sdpvar(1,1);
u_max = 0.61;

for k=1:N
    % LMI 1
    X_1 = [1 x{k}';x{k} G];
    % LMI 2
    X_2 = [G         G*A'+Y'*B' G*(Q^0.5)      Y'*(R^0.5);...
           A*G+B*Y   Q          zeros(ma,ma)     zeros(ma,nb);...
           (Q^0.5)*G zeros(ma,ma) gamma*eye(ma,ma) zeros(ma,nb);...
           (R^0.5)*Y zeros(nb,na) zeros(nb,na)     gamma*eye(nb)];
    % LMI 3 - input constraints
    X_3 = [u_max^2 Y;...
            Y'      G];
    
    % Constraints
    constraints = [X_1 >= 0, X_2 >= 0, X_3 >= 0];
    % Set some options for YALMIP and solver
    % Sedumi does not give solution sometimes. Use Mosek.
    options = sdpsettings('verbose',1,'solver','sedumi');

    % Solve the problem
    sol = optimize(constraints,gamma,options);

    G_val = value(G);
    Y_val = value(Y);
    K = Y_val*inv(G_val);

    u{k} = K*x{k};
    x{k+1} = A*x{k}+ B*u{k};
end
