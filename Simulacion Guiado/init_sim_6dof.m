% =========================================================================
% TFG - INICIALIZACIÓN DEL SIMULADOR 6-DOF (ARQUITECTURA DE BUSES)
% =========================================================================
clear; clc; close all;

load('AIM9J.mat')
load("su-27.mat")
disp('>>> Iniciando carga de base de datos del Simulador 6-DOF...');

%% 1. PARÁMETROS DE SIMULACIÓN Y ENTORNO
Ts = 0.005;                % Paso de simulación del SINS (200 Hz)
t_max = 60;                % Tiempo máximo de vuelo [s]

% Coordenadas de Lanzamiento (WGS84)
lat_0 = 40.4168;           % Latitud inicial [deg]
lon_0 = -3.7038;           % Longitud inicial [deg]
alt_0 = 5000;              % Altitud inicial [m]

%% 2. CONDICIONES INICIALES DEL MISIL (Planta 6-DOF)
% Posición y Velocidad
pos_0_ned = [0; 0; -alt_0];% Posición NED inicial [m]
vel_0_body = [250; 0; 0];  % Velocidad en ejes cuerpo [u, v, w] en m/s

% Actitud (Vuelo nivelado)
roll_0 = 0; pitch_0 = 0; yaw_0 = 0;
quat_0 = angle2quat(yaw_0, pitch_0, roll_0, 'ZYX')'; % Cuaternión inicial
omega_0_body = [0; 0; 0];  % Tasas de giro [p, q, r] en rad/s

%% 3. MASA E INERCIA (AIM-9 Sidewinder)
masa_ini = 85.0;           % Masa inicial [kg]
masa_fin = 58.0;           % Masa final (motor apagado) [kg]
t_burn = 5.0;              % Tiempo de quemado [s]

% Tensor de inercia (simplificado constante para no complicar la matriz dI/dt)
Ixx = 0.5; Iyy = 60.0; Izz = 60.0;
I_tensor = diag([Ixx, Iyy, Izz]);

%% 4. MOTOR Y AERODINÁMICA 6-DOF
Empuje_Max = 16000;        % Empuje [N]

% Geometría
diametro = 0.127;          % Diámetro [m]
S_ref = pi*(diametro/2)^2; % Área de referencia [m^2]
L_ref = diametro;          % Longitud de referencia aerodinámica [m]

% Tablas Aerodinámicas Simplificadas (Dependientes del Mach)
Mach_vec = [0.0, 0.8, 1.0, 1.2, 1.5, 2.0, 3.0, 4.0];
Cd_0     = [0.4, 0.42, 0.65, 0.55, 0.45, 0.35, 0.30, 0.28]; % Drag base
Cna      = [15,  16,   18,   20,   18,   15,   12,   10  ]; % Derivada Fuerza Normal vs Alpha
Cma      = [-25, -28, -35,  -40,  -35,  -30,  -25,  -20 ];  % Derivada Momento Cabeceo vs Alpha (Estabilidad)

% Derivadas de Control (Poder de las aletas)
Cme = -15;  % Eficiencia del Elevador en Cabeceo (Pitch)
Cne = -15;  % Eficiencia del Timón en Guiñada (Yaw)

%% 5. DEFINICIÓN DE LOS BUSES DE DATOS (LA MAGIA DE LA ARQUITECTURA)
% Simulink usará estos objetos para estructurar las señales

% --- BUS ESTADO (Lo que hace el misil) ---
elems(1) = Simulink.BusElement; elems(1).Name = 'Pos_NED';    elems(1).Dimensions = 3;
elems(2) = Simulink.BusElement; elems(2).Name = 'Vel_Body';   elems(2).Dimensions = 3;
elems(3) = Simulink.BusElement; elems(3).Name = 'Quat';       elems(3).Dimensions = 4;
elems(4) = Simulink.BusElement; elems(4).Name = 'Omega_Body'; elems(4).Dimensions = 3;
elems(5) = Simulink.BusElement; elems(5).Name = 'Mach';       elems(5).Dimensions = 1;
elems(6) = Simulink.BusElement; elems(6).Name = 'Alpha';      elems(6).Dimensions = 1;
elems(7) = Simulink.BusElement; elems(7).Name = 'Beta';       elems(7).Dimensions = 1;
elems(8) = Simulink.BusElement; elems(8).Name = 'Masa';       elems(8).Dimensions = 1;
elems(9) = Simulink.BusElement; elems(9).Name = 'Accel_Body'; elems(9).Dimensions = 3;
Bus_Estado = Simulink.Bus;
Bus_Estado.Elements = elems; clear elems;

% --- BUS ENTORNO (Lo que la atmósfera le hace al misil) ---
elems(1) = Simulink.BusElement; elems(1).Name = 'Gravedad_NED'; elems(1).Dimensions = 3;
elems(2) = Simulink.BusElement; elems(2).Name = 'Densidad';     elems(2).Dimensions = 1;
elems(3) = Simulink.BusElement; elems(3).Name = 'Vel_Sonido';   elems(3).Dimensions = 1;
elems(4) = Simulink.BusElement; elems(4).Name = 'Presion';      elems(4).Dimensions = 1;
Bus_Entorno = Simulink.Bus;
Bus_Entorno.Elements = elems; clear elems;

% --- BUS COMANDOS (Lo que el Autopiloto le pide a la Planta) ---
elems(1) = Simulink.BusElement; elems(1).Name = 'Empuje_Cmd'; elems(1).Dimensions = 1;
elems(2) = Simulink.BusElement; elems(2).Name = 'Elevador';   elems(2).Dimensions = 1; % Para cabeceo (Pitch)
elems(3) = Simulink.BusElement; elems(3).Name = 'Timon';      elems(3).Dimensions = 1; % Para guiñada (Yaw)
elems(4) = Simulink.BusElement; elems(4).Name = 'Aleron';     elems(4).Dimensions = 1; % Para alabeo (Roll)
Bus_Comandos = Simulink.Bus;
Bus_Comandos.Elements = elems; clear elems;

%% 6. MODELO DE SENSORES (IMU y BARÓMETRO) - Capítulo 4
% Giróscopo (Mide tasas de giro p, q, r)
gyro_bias = [0.01; 0.01; 0.01];       % Sesgo constante [rad/s]
gyro_noise_power = [1e-5 1e-5 1e-5];  % Potencia del ruido blanco

% Barómetro / Altímetro (Mide el eje Z de Pos_NED)
% Asumimos un error constante de calibración y ruido de viento
alt_bias = [0; 0; 15];                % Error de 15 metros en el eje Z (Abajo)
alt_noise_power = [1e-6 1e-6 0.1];    % Mucho ruido en el eje Z

% Acelerómetro (Mide Fuerza Específica en X, Y, Z)
accel_bias = [0.05; 0.05; 0.05];       % Sesgo constante [m/s^2]
accel_noise_power = [1e-4 1e-4 1e-4];  % Ruido blanco

%% 7. MODELO DEL OBJETIVO (Capítulo 6 - Guiado)
% Dron enemigo a 10 km al Norte, misma altitud
pos_target_0 = [10000; 0; -5000]; % [Norte, Este, Abajo] en metros
vel_target_0 = [-250; 0; 0];      % Vuela hacia el Sur a 250 m/s

% Constante de Navegación Proporcional (Típicamente entre 3 y 5)
N_pn = 4.0;

%% 8. MODELO GNSS (GPS) - Fusión Sensorial
gnss_noise_pos = [2; 2; 3];     % Ruido del GPS (2m en horizontal, 3m en vertical)
gnss_noise_vel = [0.1; 0.1; 0.15]; % Ruido midiendo la velocidad (m/s)
Ts_gnss = 0.1;                  % El GPS es más lento: actualiza a 10 Hz (cada 0.1s)

disp('>>> [ÉXITO] Variables cargadas y Buses de Datos creados.');
disp('>>> Siguiente paso: Abrir Simulink.');