% =========================================================================
% TFG: Fusión Sensorial SINS/GPS mediante Filtro de Kalman Extendido (ESKF)
% =========================================================================
clear; clc; close all;

%% 1. CONFIGURACIÓN INICIAL
archivo = 'trajectory_tbm.mat'; % Cambia a tbm, bomb o sam según necesites
load(archivo, 'data');
data(end, :) = [];
N = size(data, 1);

%Poco tecnico, graficas de errores pintar cada uno de los errores antes y
%despues del filtro y pintar los errores. histograma de ese ruido normales
%centrados en cero.


% Configuración dinámica de frecuencias (IMU vs GPS)
switch archivo
    case 'trajectory_tbm.mat'
        dt = 0.05;       dt_gps = 1.22461;  % GPS a 1 Hz
    case 'trajectory_aam.mat'
        dt = 0.0001;     dt_gps = 0.01; % GPS a 100 Hz
    case 'trajectory_bomb.mat'
        dt = 0.1;        dt_gps = 0.01;  % GPS a 2 Hz
    case 'trajectory_sam.mat'
        dt = 0.0001;     dt_gps = 0.01; % GPS a 100 Hz
    otherwise
        error('Archivo no reconocido');
end

t = (0:N-1)' * dt;
g_n = [0; 0; -9.81]; % Gravedad corregida (Z-Up)

%% 2. EXTRACCIÓN Y CINEMÁTICA (Verdad)
% IMPORTANTE: El archivo data contiene = [x, y, z, pitch, yaw, roll]
pos_true = data(:, 1:3);

% Reordenamos internamente a [roll, pitch, yaw]
euler_true = [data(:, 6), data(:, 4), data(:, 5)];

vel_true = zeros(N, 3); acc_true = zeros(N, 3); euler_rates = zeros(N, 3);
for i = 1:3
    vel_true(:, i) = gradient(pos_true(:, i), dt);
    acc_true(:, i) = gradient(vel_true(:, i), dt);
    euler_rates(:, i) = gradient(euler_true(:, i), dt);
end

f_b_true = zeros(N, 3); w_b_true = zeros(N, 3);
for k = 1:N
    phi = euler_true(k, 1); theta = euler_true(k, 2); psi = euler_true(k, 3);
    dphi = euler_rates(k,1); dtheta = euler_rates(k,2); dpsi = euler_rates(k,3);
    w_b_true(k, :) = [dphi - dpsi*sin(theta), dtheta*cos(phi) + dpsi*cos(theta)*sin(phi), -dtheta*sin(phi) + dpsi*cos(theta)*cos(phi)];
    
    C_n_b = [1 0 0; 0 cos(phi) sin(phi); 0 -sin(phi) cos(phi)] * ...
            [cos(theta) 0 -sin(theta); 0 1 0; sin(theta) 0 cos(theta)] * ...
            [cos(psi) sin(psi) 0; -sin(psi) cos(psi) 0; 0 0 1];
    f_b_true(k, :) = (C_n_b * (acc_true(k, :)' - g_n))';
end

%% 3. MODELO DE SENSORES (IMU y GPS)
rng(42);
bias_accel = [0.05; -0.05; 0.02]; noise_accel = 0.01;
bias_gyro = [0.1; -0.1; 0.05] * (pi/180); noise_gyro = 0.005 * (pi/180);

f_b_meas = f_b_true + bias_accel' + (noise_accel/sqrt(dt))*randn(N, 3);
w_b_meas = w_b_true + bias_gyro'  + (noise_gyro/sqrt(dt))*randn(N, 3);

% Simulación de GPS Ruidoso
std_gps = 3.0; % 3 metros de error típico GPS
pos_gps = pos_true + std_gps * randn(N, 3);

%% 4. INICIALIZACIÓN DE NAVEGADORES Y FILTRO
% Variables para INS Puro (Deriva libre)
pos_ins_pure = zeros(N, 3); vel_ins_pure = zeros(N, 3); q_ins_pure = zeros(N, 4);
pos_ins_pure(1,:) = pos_true(1,:); vel_ins_pure(1,:) = vel_true(1,:);
q_ins_pure(1,:) = euler2quat(euler_true(1,1), euler_true(1,2), euler_true(1,3));

% Variables para INS + GPS (EKF)
pos_ekf = zeros(N, 3); vel_ekf = zeros(N, 3); q_ekf = zeros(N, 4);
pos_ekf(1,:) = pos_true(1,:); vel_ekf(1,:) = vel_true(1,:);
q_ekf(1,:) = q_ins_pure(1,:);

est_bias_a = zeros(3,1); % El filtro arranca sin conocer los sesgos
est_bias_g = zeros(3,1);

% Matrices del Filtro de Kalman
P = eye(15) * 0.1; % Matriz de Covarianza Inicial
P(10:12, 10:12) = eye(3) * (0.1)^2;  % Incertidumbre inicial bias acelerómetro
P(13:15, 13:15) = eye(3) * (0.1*pi/180)^2; % Incertidumbre inicial bias giróscopo

% Ruido de Proceso (Q) y Medida (R)
Q = diag([zeros(1,3), ones(1,3)*(noise_accel^2), ones(1,3)*(noise_gyro^2), zeros(1,6)]);
R = eye(3) * (std_gps^2);
H = [eye(3), zeros(3, 12)]; % El GPS solo mide posición (estados 1 al 3)

P_hist = zeros(N, 15); P_hist(1,:) = diag(P)';
gps_update_flag = false(N,1);

%% 5. BUCLE PRINCIPAL DE NAVEGACIÓN Y ESTIMACIÓN
tiempo_proximo_gps = dt_gps;

for k = 1:N-1
    % =========================================================
    % A. NAVEGACIÓN INERCIAL PURA (Sin corrección, para comparar)
    % =========================================================
    q_ins_pure(k+1,:) = prop_quat(q_ins_pure(k,:), w_b_meas(k,:)', dt);
    C_b_n_pure = quat2dcm(q_ins_pure(k+1,:));
    a_n_pure = C_b_n_pure * f_b_meas(k,:)' + g_n;
    vel_ins_pure(k+1,:) = vel_ins_pure(k,:) + a_n_pure' * dt;
    pos_ins_pure(k+1,:) = pos_ins_pure(k,:) + vel_ins_pure(k,:)*dt + 0.5*a_n_pure'*dt^2;
    
    % =========================================================
    % B. MECANIZACIÓN INS PARA EL EKF (Con compensación de bias)
    % =========================================================
    w_b_corr = w_b_meas(k,:)' - est_bias_g;
    f_b_corr = f_b_meas(k,:)' - est_bias_a;
    
    q_ekf(k+1,:) = prop_quat(q_ekf(k,:), w_b_corr, dt);
    C_b_n = quat2dcm(q_ekf(k+1,:));
    f_n = C_b_n * f_b_corr;
    a_n = f_n + g_n;
    
    vel_ekf(k+1,:) = vel_ekf(k,:) + a_n' * dt;
    pos_ekf(k+1,:) = pos_ekf(k,:) + vel_ekf(k,:)*dt + 0.5*a_n'*dt^2;
    
    % =========================================================
    % C. PREDICCIÓN DEL FILTRO DE KALMAN (Ecuaciones de Error SINS)
    % =========================================================
    F = zeros(15, 15);
    F(1:3, 4:6) = eye(3);                     % dPos/dt = dVel
    F(4:6, 7:9) = -skew(f_n);                 % dVel/dt = -[f_n x] * dPhi
    F(4:6, 10:12) = C_b_n;                    % Error por bias acelerómetro
    F(7:9, 13:15) = -C_b_n;                   % Error por bias giróscopo
    
    Phi = eye(15) + F * dt;
    P = Phi * P * Phi' + Q * dt;
    
    % =========================================================
    % D. ACTUALIZACIÓN GPS (Corrección)
    % =========================================================
    if t(k+1) >= tiempo_proximo_gps
        gps_update_flag(k+1) = true;
        tiempo_proximo_gps = tiempo_proximo_gps + dt_gps;
        
        z = pos_ekf(k+1,:)' - pos_gps(k+1,:)'; 
        
        S = H * P * H' + R;
        K = P * H' / S;
        
        dx = K * z;
        P = (eye(15) - K * H) * P;
        
        % Feedback al INS (Bucle Cerrado)
        pos_ekf(k+1,:) = pos_ekf(k+1,:) - dx(1:3)';
        vel_ekf(k+1,:) = vel_ekf(k+1,:) - dx(4:6)';
        
        dphi = dx(7:9);
        dq = [1, -dphi(1)/2, -dphi(2)/2, -dphi(3)/2];
        q_ekf(k+1,:) = quat_mult(dq, q_ekf(k+1,:));
        q_ekf(k+1,:) = q_ekf(k+1,:) / norm(q_ekf(k+1,:));
        
        est_bias_a = est_bias_a + dx(10:12);
        est_bias_g = est_bias_g + dx(13:15);
    end
    
    P_hist(k+1,:) = diag(P)';
end

%% 6. RESULTADOS PARA EL TFG
figure('Name', 'Comparativa 3D: Verdad vs EKF', 'Color', 'w');
plot3(pos_true(:,1), pos_true(:,2), pos_true(:,3), 'b', 'LineWidth', 2); hold on;
plot3(pos_ins_pure(:,1), pos_ins_pure(:,2), pos_ins_pure(:,3), 'r--', 'LineWidth', 1.5);
plot3(pos_ekf(:,1), pos_ekf(:,2), pos_ekf(:,3), 'g', 'LineWidth', 2);
plot3(pos_gps(gps_update_flag,1), pos_gps(gps_update_flag,2), pos_gps(gps_update_flag,3), 'k.', 'MarkerSize', 8);
grid on; axis equal; xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z (Altitud) [m]');
title(['Fusión INS/GPS (EKF) - ', archivo]);
legend('Verdad', 'INS (Deriva Pura)', 'SINS+GPS (EKF)', 'Medidas GPS', 'Location', 'Best');

figure('Name', 'Error de Navegación y Filtro', 'Color', 'w', 'Position', [100 100 800 600]);
error_pure = sqrt(sum((pos_ins_pure - pos_true).^2, 2));
error_ekf = sqrt(sum((pos_ekf - pos_true).^2, 2));

subplot(2,1,1);
plot(t, error_pure, 'r', 'LineWidth', 1.5); hold on;
plot(t, error_ekf, 'g', 'LineWidth', 1.5);
grid on; ylabel('Error Euclidiano [m]'); title('Comparativa de Error de Posición');
legend('INS Puro', 'EKF (INS+GPS)', 'Location', 'Best');

subplot(2,1,2);
% Gráfico con los límites 3-Sigma del Filtro (Solo eje X como ejemplo)
err_x = pos_ekf(:,1) - pos_true(:,1);
sigma3_x = 3 * sqrt(P_hist(:,1));
plot(t, err_x, 'b', 'LineWidth', 1.5); hold on;
plot(t, sigma3_x, 'r--', t, -sigma3_x, 'r--', 'LineWidth', 1);
grid on; xlabel('Tiempo [s]'); ylabel('Error en X [m]');
title('Error Real Eje X vs Límites 3-\sigma del Filtro de Kalman');
legend('Error real EKF', '\pm 3\sigma Covarianza');

%% FUNCIONES AUXILIARES
function S = skew(v)
    S = [0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0];
end

function q_next = prop_quat(q, w, dt)
    norm_w = norm(w);
    if norm_w > 1e-12
        alpha = norm_w * dt; axis = w / norm_w;
        dq = [cos(alpha/2); axis * sin(alpha/2)]';
    else
        dq = [1, 0, 0, 0];
    end
    q_next = quat_mult(q, dq);
    q_next = q_next / norm(q_next);
end

function q = euler2quat(phi, theta, psi)
    c1 = cos(phi/2); s1 = sin(phi/2); c2 = cos(theta/2); s2 = sin(theta/2); c3 = cos(psi/2); s3 = sin(psi/2);
    q = [c1*c2*c3 + s1*s2*s3, s1*c2*c3 - c1*s2*s3, c1*s2*c3 + s1*c2*s3, c1*c2*s3 - s1*s2*c3];
end

function C = quat2dcm(q)
    q0=q(1); q1=q(2); q2=q(3); q3=q(4);
    C = [q0^2+q1^2-q2^2-q3^2, 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2);
         2*(q1*q2+q0*q3), q0^2-q1^2+q2^2-q3^2, 2*(q2*q3-q0*q1);
         2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), q0^2-q1^2-q2^2+q3^2];
end

function qr = quat_mult(p, q)
    p0=p(1); p1=p(2); p2=p(3); p3=p(4); q0=q(1); q1=q(2); q2=q(3); q3=q(4);
    qr = [p0*q0-p1*q1-p2*q2-p3*q3, p0*q1+p1*q0+p2*q3-p3*q2, p0*q2-p1*q3+p2*q0+p3*q1, p0*q3+p1*q2-p2*q1+p3*q0];
end