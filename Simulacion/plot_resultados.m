%% TFG Alejandro Burgos Alemany
clc; close all;

% --- 1. GRUND TRUTH ---
if exist('out', 'var') && (isprop(out, 'out_Pos_Verdad') || isfield(out, 'out_Pos_Verdad'))
    data_true = out.out_Pos_Verdad;
    t_global  = out.tout;
elseif exist('out_Pos_Verdad', 'var')
    data_true = out_Pos_Verdad;
    if exist('tout', 'var'), t_global = tout; else, t_global = []; end
else
    error('Error.');
end

% --- 2. ESTIMACION ---
if exist('out', 'var') && (isprop(out, 'out_Pos_Est') || isfield(out, 'out_Pos_Est'))
    data_est = out.out_Pos_Est;
elseif exist('out_Pos_Est', 'var')
    data_est = out_Pos_Est;
else
    error('Error.');
end


% --- Extracción de tiempos y valores (Verdad) ---
if isfield(data_true, 'time') || isprop(data_true, 'time'), t_true = data_true.time;
elseif isfield(data_true, 'Time') || isprop(data_true, 'Time'), t_true = data_true.Time;
else, t_true = t_global; end

if isfield(data_true, 'signals'), p_true_raw = data_true.signals.values;
elseif isfield(data_true, 'Data') || isprop(data_true, 'Data'), p_true_raw = data_true.Data;
else, p_true_raw = data_true; end

% --- Extracción de tiempos y valores (Estimación) ---
if isfield(data_est, 'time') || isprop(data_est, 'time'), t_est = data_est.time;
elseif isfield(data_est, 'Time') || isprop(data_est, 'Time'), t_est = data_est.Time;
else, t_est = t_global; end

if isfield(data_est, 'signals'), p_est_raw = data_est.signals.values;
elseif isfield(data_est, 'Data') || isprop(data_est, 'Data'), p_est_raw = data_est.Data;
else, p_est_raw = data_est; end

% --- LIMPIEZA Y FORMATEO MATRICIAL ---
p_true_raw = squeeze(p_true_raw);
p_est_raw  = squeeze(p_est_raw);

if size(p_true_raw, 1) == 3 && size(p_true_raw, 2) > 3, pos_true = p_true_raw'; else, pos_true = p_true_raw; end
if size(p_est_raw, 1) == 3 && size(p_est_raw, 2) > 3, pos_est = p_est_raw'; else, pos_est = p_est_raw; end

% --- SINCRONIZACIÓN AUTOMÁTICA ---
if length(t_true) ~= length(t_est)
    pos_est_sync = zeros(length(t_true), 3);
    pos_est_sync(:,1) = interp1(t_est, pos_est(:,1), t_true, 'linear', 'extrap');
    pos_est_sync(:,2) = interp1(t_est, pos_est(:,2), t_true, 'linear', 'extrap');
    pos_est_sync(:,3) = interp1(t_est, pos_est(:,3), t_true, 'linear', 'extrap');
    pos_est = pos_est_sync;
end
t = t_true;


%% GRÁFICA 1: TRAYECTORIA ESPACIAL 3D
figure('Name', 'Trayectoria Espacial 3D', 'Color', 'w', 'Position', [100 100 900 600]);
plot3(pos_true(:,2), pos_true(:,1), zeros(length(t),1), '-', 'Color', [0.8 0.8 0.8], 'LineWidth', 2); hold on;
plot3(pos_true(:,2), pos_true(:,1), -pos_true(:,3), 'k-', 'LineWidth', 2); 
plot3(pos_est(:,2), pos_est(:,1), -pos_est(:,3), 'r--', 'LineWidth', 1.5);
axis equal; grid on; grid minor; view(-35, 30);
xlabel('Desvío Lateral (Este) [m]', 'FontWeight', 'bold'); 
ylabel('Avance Longitudinal (Norte) [m]', 'FontWeight', 'bold'); 
zlabel('Altitud [m]', 'FontWeight', 'bold');
legend('Proyección en tierra', 'Verdad Terreno (Física)', 'Navegación Estimada (ES-EKF)', 'Location', 'best');
title('Caso de Estudio: Perfil de Vuelo del Misil (Simulink)');
hold off;

%% GRÁFICA 2: ANÁLISIS DEL ERROR DE NAVEGACIÓN
figure('Name', 'Errores del ES-EKF', 'Color', 'w', 'Position', [150 150 800 500]);
error_pos = pos_true - pos_est;
plot(t, error_pos(:,1), 'b', 'LineWidth', 1.2); hold on;
plot(t, error_pos(:,2), 'r', 'LineWidth', 1.2);
plot(t, error_pos(:,3), 'g', 'LineWidth', 1.2);
grid on; grid minor;
xlabel('Tiempo de vuelo [s]', 'FontWeight', 'bold'); 
ylabel('Error de Posición [m]', 'FontWeight', 'bold');
legend('Error Norte (\deltaN)', 'Error Este (\deltaE)', 'Error Altitud (\deltaD)', 'Location', 'best');
title('Convergencia del Error de Navegación en Lazo Cerrado');