% =========================================================================
% TFG: Generación y Análisis de Trayectorias SINS
% =========================================================================
clear; clc; close all;

%% 1. SELECCIÓN DE TRAYECTORIA Y CONFIGURACIÓN DE TIEMPO
% Descomenta la que quieras probar:
archivo = 'trajectory_bomb.mat';   % Misil Aire-Aire (~3.6s)
% archivo = 'trajectory_tbm.mat';  % Misil Balístico (~175s)
% archivo = 'trajectory_bomb.mat'; % Aire-Tierra (~6s)
% archivo = 'trajectory_sam.mat';  % Superficie-Aire (~1s)

load(archivo, 'data');
data(end, :) = [];
N = size(data, 1);

% Selección dinámica del dt
switch archivo
    case 'trajectory_tbm.mat'
        dt = 0.05;
    case 'trajectory_aam.mat'
        dt = 0.0001; 
    case 'trajectory_bomb.mat'
        dt = 0.1;
    case 'trajectory_sam.mat'
        dt = 0.0001;
    otherwise
        error('Archivo no reconocido para asignación de dt');
end

t_final = (N-1) * dt;
t = (0:N-1)' * dt; 

fprintf('Simulando trayectoria: %s\n', archivo);
fprintf('Duración real: %.4f segundos | Puntos: %d\n', t_final, N);

%% 2. EXTRACCIÓN DE LA VERDAD (Input Data)
% IMPORTANTE: El archivo data contiene = [x, y, z, pitch, yaw, roll]
pos_true = data(:, 1:3);

% data(:,6) = roll, data(:,4) = pitch, data(:,5) = yaw
euler_true = [data(:, 6), data(:, 4), data(:, 5)];

%% 3. INVERSIÓN CINEMÁTICA (Generación de medidas de IMU)
% Derivamos para obtener velocidad y aceleración
vel_true = zeros(N, 3);
acc_true = zeros(N, 3);
for i = 1:3
    vel_true(:, i) = gradient(pos_true(:, i), dt);
    acc_true(:, i) = gradient(vel_true(:, i), dt);
end

euler_rates = zeros(N, 3);
for i = 1:3
    euler_rates(:, i) = gradient(euler_true(:, i), dt);
end

% GRAVEDAD CORREGIDA: Z positivo es hacia arriba (Altitud)
g_n = [0; 0; -9.81]; 

f_b_true = zeros(N, 3);
w_b_true = zeros(N, 3);

for k = 1:N
    % Extrae los ángulos
    phi = euler_true(k, 1); theta = euler_true(k, 2); psi = euler_true(k, 3);
    dphi = euler_rates(k, 1); dtheta = euler_rates(k, 2); dpsi = euler_rates(k, 3);
    
    % Euler rates -> Body rates
    p = dphi - dpsi * sin(theta);
    q = dtheta * cos(phi) + dpsi * cos(theta) * sin(phi);
    r = -dtheta * sin(phi) + dpsi * cos(theta) * cos(phi);
    w_b_true(k, :) = [p, q, r];
    
    % DCM Nav to Body (Ejes ENU/Cartesianos)
    C_yaw   = [cos(psi) sin(psi) 0; -sin(psi) cos(psi) 0; 0 0 1];
    C_pitch = [cos(theta) 0 -sin(theta); 0 1 0; sin(theta) 0 cos(theta)];
    C_roll  = [1 0 0; 0 cos(phi) sin(phi); 0 -sin(phi) cos(phi)];
    C_n_b = C_roll * C_pitch * C_yaw;
    
    % Fuerza específica: f = a - g. 
    f_n = acc_true(k, :)' - g_n;
    f_b_true(k, :) = (C_n_b * f_n)';
end

%% 4. MODELO DE ERRORES IMU

%Variacion sobre mismo codigo o repetir varais en func inercial, repetir
%como el q tendria en movil/avion/barco

bias_accel = [0.05; -0.05; 0.02];   
noise_accel = 0.01;                 
bias_gyro = [0.1; -0.1; 0.05] * (pi/180); 
noise_gyro = 0.005 * (pi/180);            

rng(42); 
f_b_meas = f_b_true + bias_accel' + (noise_accel / sqrt(dt)) * randn(N, 3);
w_b_meas = w_b_true + bias_gyro' + (noise_gyro / sqrt(dt)) * randn(N, 3);

%% 5. MECANIZACIÓN SINS (Integración)
pos_ins = zeros(N, 3);
vel_ins = zeros(N, 3);
q_ins = zeros(N, 4); 

pos_ins(1, :) = pos_true(1, :);
vel_ins(1, :) = vel_true(1, :);
q_ins(1, :) = euler2quat(euler_true(1, 1), euler_true(1, 2), euler_true(1, 3));

for k = 1:N-1
    % Propagación Actitud (Cuaterniones)
    wb = w_b_meas(k, :)';
    norm_w = norm(wb);
    if norm_w > 1e-12
        alpha = norm_w * dt;
        eje_rot = wb / norm_w;
        dq = [cos(alpha/2); eje_rot * sin(alpha/2)]';
    else
        dq = [1, 0, 0, 0];
    end
    q_ins(k+1, :) = quat_mult(q_ins(k, :), dq);
    q_ins(k+1, :) = q_ins(k+1, :) / norm(q_ins(k+1, :));
    
    % Integración Velocidad y Posición
    C_b_n = quat2dcm(q_ins(k+1, :)); 
    f_n_meas = C_b_n * f_b_meas(k, :)';
    a_n_meas = f_n_meas + g_n; % Sumamos gravedad (-9.81)
    
    vel_ins(k+1, :) = vel_ins(k, :) + a_n_meas' * dt;
    pos_ins(k+1, :) = pos_ins(k, :) + vel_ins(k, :) * dt + 0.5 * a_n_meas' * dt^2;
end

%% 6. GRÁFICAS DE RESULTADOS (Tiempo real corregido)
% Trayectoria 3D
figure('Name', 'Visualización 3D', 'Color', 'w');
plot3(pos_true(:,1), pos_true(:,2), pos_true(:,3), 'b', 'LineWidth', 2); hold on;
plot3(pos_ins(:,1), pos_ins(:,2), pos_ins(:,3), 'r--', 'LineWidth', 1.5);
grid on; axis equal;
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z (Altitud) [m]');
title(['Trayectoria: ', archivo, ' (Duración: ', num2str(t_final), 's)']);
legend('Verdad', 'INS con Errores', 'Location', 'Best');
view(45, 30);

% Error de deriva (Eje de tiempo corregido)
figure('Name', 'Análisis de Deriva', 'Color', 'w');
error_pos = pos_ins - pos_true;
subplot(2,1,1);
plot(t, error_pos, 'LineWidth', 1.5);
grid on;
title(['Evolución del Error de Navegación (', archivo, ')']);
ylabel('Error Posición [m]'); xlabel('Tiempo [s]');
legend('Error X', 'Error Y', 'Error Z');

subplot(2,1,2);
dist_error = sqrt(sum(error_pos.^2, 2));
plot(t, dist_error, 'k', 'LineWidth', 1.5);
grid on;
ylabel('Error Total (m)'); xlabel('Tiempo [s]');
legend('Distancia Euclidiana');

%% FUNCIONES AUXILIARES
function q = euler2quat(phi, theta, psi)
    c1 = cos(phi/2); s1 = sin(phi/2);
    c2 = cos(theta/2); s2 = sin(theta/2);
    c3 = cos(psi/2); s3 = sin(psi/2);
    q = [c1*c2*c3 + s1*s2*s3, s1*c2*c3 - c1*s2*s3, c1*s2*c3 + s1*c2*s3, c1*c2*s3 - s1*s2*c3];
end

function C = quat2dcm(q)
    q0 = q(1); q1 = q(2); q2 = q(3); q3 = q(4);
    C = [q0^2+q1^2-q2^2-q3^2, 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2);
         2*(q1*q2+q0*q3), q0^2-q1^2+q2^2-q3^2, 2*(q2*q3-q0*q1);
         2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), q0^2-q1^2-q2^2+q3^2];
end

function qr = quat_mult(p, q)
    p0 = p(1); p1 = p(2); p2 = p(3); p3 = p(4);
    q0 = q(1); q1 = q(2); q2 = q(3); q3 = q(4);
    qr = [p0*q0 - p1*q1 - p2*q2 - p3*q3, ...
          p0*q1 + p1*q0 + p2*q3 - p3*q2, ...
          p0*q2 - p1*q3 + p2*q0 + p3*q1, ...
          p0*q3 + p1*q2 - p2*q1 + p3*q0];
end