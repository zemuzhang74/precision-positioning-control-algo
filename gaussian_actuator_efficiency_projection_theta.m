clc; clear; close all;

%% Parameters
max_iters = 15;
tol = 0.5;

pos = [50 * rand(2,1); 0.1*randn];   % [x; y; theta]
ref = [0; 0; 0];

R = 10;
gamma = 0.2;

angles = [0,120,240]*pi/180;

theta_plot = linspace(0,2*pi,100);

%% ✅ Adaptive variable
alpha = 1.0;

%% Visualization
figure; axis equal;
xlim([-150 150]); ylim([-150 150]);
grid on; hold on;
title('Orientation-Dependent Contact Model (J = J(theta))');

h_circle = plot(0,0,'b','LineWidth',2);
h_error  = quiver(0,0,0,0,0,'r');
h_text   = text(-140,130,'');

%% Loop
for k = 1:max_iters
    
    theta = pos(3);
    
    %% ✅ Compute rotation matrix of object
    c = cos(theta);
    s = sin(theta);
    Rot = [c -s; s c];
    
    %% ✅ Build J(theta) dynamically
    J = zeros(3,3);
    
    for i = 1:3
        
        % Contact point in body frame
        p_body = [R*cos(angles(i)); R*sin(angles(i))];
        
        % Rotate to world frame
        p_world = Rot * p_body;
        
        % Force direction (actuator direction in world frame)
        dir = [cos(angles(i)); sin(angles(i))];
        
        % Fill Jacobian column
        J(1:2,i) = dir;   % linear motion
        
        % torque = cross product (2D: x*y - y*x)
        J(3,i) = p_world(1)*dir(2) - p_world(2)*dir(1);
    end
    
    %% ✅ ERROR (ignore theta if desired)
    e = pos - ref;
    e(3) = 0;   % still ignoring orientation control
    
    if norm(e(1:2)) < tol
        disp(['✅ Converged in ', num2str(k)]);
        break;
    end
    
    %% ✅ CONTROL (electrical input)
    gain = 1 / alpha;
    u = -gain * (J \ e);
    
    %% ✅ REAL ACTUATOR
    K_true = normrnd(0.9, 0.05, [3,1]);
    K_true = max(min(K_true,1.2),0.7);
    
    F_actual = K_true .* u;
    
    %% ✅ APPLY MOTION
    pos_old = pos;
    pos = pos + J * F_actual;
    
    %% ✅ ADAPTIVE UPDATE
    expected = J * u;
    actual   = pos - pos_old;
    
    if norm(expected) > 1e-8
        alpha_new = dot(actual, expected) / norm(expected)^2;
        alpha = (1 - gamma)*alpha + gamma*alpha_new;
    end
    
    %% Visualization
    circle_pts = Rot * [R*cos(theta_plot); R*sin(theta_plot)];
    
    set(h_circle,...
        'XData',pos(1)+circle_pts(1,:),...
        'YData',pos(2)+circle_pts(2,:));
    
    set(h_error,...
        'XData',pos(1),...
        'YData',pos(2),...
        'UData',-pos(1),...
        'VData',-pos(2));
    
    set(h_text,'String',...
        sprintf('Iter: %d\nalpha: %.3f\ntheta: %.2f',...
        k, alpha, theta));
    
    drawnow;
    pause(0.6);
end

disp(['Final position error: ', num2str(norm(pos(1:2)))]);
