%% TFG Alejandro Burgos Alemany

clear; clc; close all;

%% 1. PARÁMETROS DE SIMULACIÓN Y ENTORNO

t_sim = 60; % Tiempo total de simulación [s]

dt_imu = 0.005; % Paso de integración IMU (200 Hz)

dt_gps = 0.1; % Paso de actualización GPS (10 Hz)

sim_year = 2025; % Año para modelos geomagnéticos/gravedad

%% 2. CINEMÁTICA INICIAL (Planta 6-DOF)

% Posición inicial (Lat, Lon, Alt) - NED (Z es negativo hacia arriba)

lat_0 = 40.4168;

lon_0 = -3.7038;

alt_0 = 0; % Altitud [m]

Pos_0_NED = [lat_0; lon_0; -alt_0];

% Velocidad inicial en ejes cuerpo [u, v, w] [m/s]

V_body_0 = [0; 0; 0];

% Actitud inicial (Roll, Pitch, Yaw) [rad]

roll_rad = deg2rad(0);

pitch_rad = deg2rad(20);

yaw_rad = deg2rad(45);

Euler_0 = [roll_rad, pitch_rad, yaw_rad]; % Para el bloque 6DOF de Simulink

quat_0 = angle2quat(yaw_rad, pitch_rad, roll_rad, 'ZYX'); % Para el filtro

% Tasas angulares iniciales [p, q, r] [rad/s]

omega_0 = [0; 0; 0];

%% 3. DINÁMICA DE MASA E INERCIA

masa_0 = 150; % Masa inicial [kg]

Ixx = 2.5; % Inercia Roll

Iyy = 60.0; % Inercia Pitch

Izz = 60.0; % Inercia Yaw

I_tensor = diag([Ixx, Iyy, Izz]); % Matriz 3x3 constante

%% 4. FUERZAS SIMPLIFICADAS (Motor y Aero)

Empuje_Max = 8000; % Fuerza del motor [N] (Eje X cuerpo)

t_burn = 10; % Tiempo de quemado del motor [s]

Cd_A = 0.5; % Coeficiente aerodinámico simplificado * Área

%% 5. PARÁMETROS DE LOS SENSORES VIRTUALES

% IMU de Grado Táctico

imu_gyro_bias = (1 / 3600) * (pi / 180); % 1 deg/hr a rad/s

imu_accel_bias = 1 * 9.81 / 1000; % 1 mg a m/s^2

imu_gyro_noise = 0.001; % Densidad de ruido

imu_accel_noise= 0.005;

% GNSS y Altímetro

gps_pos_noise = 2.0; % Desviación estándar horizontal [m]

alt_noise = 5.0; % Desviación estándar vertical [m]
