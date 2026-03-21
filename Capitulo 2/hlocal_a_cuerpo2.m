%% Grado en Ingeniería Aeroespacial - UCLM
% TFG - Generación y análisis de trayectorias de vuelo basadas en
% navegación inercial para misiles
% Alumno - Alejandro Burgos Alemany
% Tutor - Miguel Ánguel Gömez Lopez

clear; clc; close all;

%% Estilo optimizado para exportación
set(groot,'defaultAxesFontName','Times New Roman')
set(groot,'defaultTextFontName','Times New Roman')
set(groot,'defaultAxesFontSize', 14) 
set(groot,'defaultAxesLineWidth', 1.2)

%% Cargar misil
load('AIM9J.mat')
CG = mean(V,1);
V  = V - CG;
V0 = V;

%% Ángulos
psi   = deg2rad(30);
theta = deg2rad(-20);
phi   = deg2rad(25);

%% Matrices de transformación
Rz = [ cos(psi)  sin(psi) 0;
      -sin(psi)  cos(psi) 0;
       0         0        1];
   
Ry = [ cos(theta) 0 -sin(theta);
       0          1  0;
       sin(theta) 0  cos(theta)];
   
Rx = [1 0 0;
      0 cos(phi) sin(phi);
      0 -sin(phi) cos(phi)];

%% Sistema ejes horizonte local
xh = [-1 0 0]';
yh = [ 0 1 0]';
zh = [ 0 0 -1]';
L = max(range(V))*0.5;

%% Parámetros visuales del arco
R_arc = L * 0.65; % Radio de los arcos
N_arc = 20;       % Resolución del arco
tam_flecha = L * 0.15; % Tamaño fijo para garantizar que la flecha se VEA

%% =========================================================
%% Figura
%% =========================================================
figure('Color','w','Position',[100 200 1500 450])
tiledlayout(1,3,'Padding','compact','TileSpacing','compact')

%% =========================================================
%% 1. Guiñada (Eje anterior: h -> Eje nuevo: 1)
%% =========================================================
nexttile
hold on; axis equal; axis off
view(35,20)
R1 = Rz;
V1 = (R1*V0')';
patch('Vertices',V1,'Faces',F, 'FaceColor',[0.5 0.5 0.5], 'EdgeColor','none', 'FaceLighting','gouraud');
material dull; camlight headlight; lighting gouraud

% Ejes anteriores (Negro)
quiver3(0,0,0,L*xh(1),L*xh(2),L*xh(3),'k','LineWidth',1,'MaxHeadSize',0.1)
quiver3(0,0,0,L*yh(1),L*yh(2),L*yh(3),'k','LineWidth',1,'MaxHeadSize',0.1)
quiver3(0,0,0,L*zh(1),L*zh(2),L*zh(3),'k','LineWidth',1,'MaxHeadSize',0.1)

% Ejes nuevos (Color)
x1 = R1*xh; y1 = R1*yh; z1 = zh;
quiver3(0,0,0,L*x1(1),L*x1(2),L*x1(3),'Color',[0.6 0 0],'LineWidth',1.2,'MaxHeadSize',0.1)
quiver3(0,0,0,L*y1(1),L*y1(2),L*y1(3),'Color',[0 0.5 0],'LineWidth',1.2,'MaxHeadSize',0.1)
quiver3(0,0,0,L*z1(1),L*z1(2),L*z1(3),'Color',[0 0 0.6],'LineWidth',1.2,'MaxHeadSize',0.1)

% Dibujo del Arco
t_ang = linspace(0, psi, N_arc);
arc1 = zeros(3, N_arc);
for i = 1:N_arc
    Rt = [cos(t_ang(i)) sin(t_ang(i)) 0; -sin(t_ang(i)) cos(t_ang(i)) 0; 0 0 1];
    arc1(:,i) = Rt * xh * R_arc;
end
plot3(arc1(1,:), arc1(2,:), arc1(3,:), 'k-', 'LineWidth', 1.5)

% Punta de flecha robusta (Garantizada para verse)
dir1 = arc1(:, end) - arc1(:, end-1);
dir1 = dir1 / norm(dir1); % Vector unitario tangente
P_start1 = arc1(:, end) - dir1 * tam_flecha; % Retrocedemos para que termine justo en el final
quiver3(P_start1(1), P_start1(2), P_start1(3), dir1(1)*tam_flecha, dir1(2)*tam_flecha, dir1(3)*tam_flecha, 0, 'k', 'LineWidth', 1.5, 'MaxHeadSize', 0.8);

pt_txt = arc1(:, round(N_arc/2)) * 1.25;
text(pt_txt(1), pt_txt(2), pt_txt(3), '$\psi$', 'Interpreter','latex','FontSize',18,'HorizontalAlignment','center');

t1 = text(L*xh(1),L*xh(2),L*xh(3),'$\mathbf{x}_h$','Interpreter','latex','Margin',2);
t2 = text(L*yh(1),L*yh(2),L*yh(3),'$\mathbf{y}_h$','Interpreter','latex','Margin',2);
t3 = text(L*zh(1),L*zh(2),L*zh(3),'$\mathbf{z}_h=\mathbf{z}_1$','Interpreter','latex','Margin',2);
t4 = text(L*x1(1),L*x1(2),L*x1(3),'$\mathbf{x}_1$','Interpreter','latex','Margin',2);
t5 = text(L*y1(1),L*y1(2),L*y1(3),'$\mathbf{y}_1$','Interpreter','latex','Margin',2);
axis tight; camzoom(1.3);

%% =========================================================
%% 2. Asiento (Eje anterior: 1 -> Eje nuevo: 2)
%% =========================================================
nexttile
hold on; axis equal; axis off
view(35,20)
R2 = R1*Ry;
V2 = (R2*V0')';
patch('Vertices',V2,'Faces',F, 'FaceColor',[0.5 0.5 0.5], 'EdgeColor','none', 'FaceLighting','gouraud');
material dull; camlight headlight; lighting gouraud

% Ejes anteriores (x1, y1, z1 en Negro)
quiver3(0,0,0,L*x1(1),L*x1(2),L*x1(3),'k','LineWidth',1,'MaxHeadSize',0.1)
quiver3(0,0,0,L*y1(1),L*y1(2),L*y1(3),'k','LineWidth',1,'MaxHeadSize',0.1)
quiver3(0,0,0,L*z1(1),L*z1(2),L*z1(3),'k','LineWidth',1,'MaxHeadSize',0.1)

% Ejes nuevos (Color)
x2 = R2*xh; y2 = y1; z2 = R2*zh;
quiver3(0,0,0,L*x2(1),L*x2(2),L*x2(3),'Color',[0.6 0 0],'LineWidth',1.2,'MaxHeadSize',0.1)
quiver3(0,0,0,L*y2(1),L*y2(2),L*y2(3),'Color',[0 0.5 0],'LineWidth',1.2,'MaxHeadSize',0.1)
quiver3(0,0,0,L*z2(1),L*z2(2),L*z2(3),'Color',[0 0 0.6],'LineWidth',1.2,'MaxHeadSize',0.1)

% Dibujo del Arco
t_ang = linspace(0, theta, N_arc);
arc2 = zeros(3, N_arc);
for i = 1:N_arc
    Rt = [cos(t_ang(i)) 0 -sin(t_ang(i)); 0 1 0; sin(t_ang(i)) 0 cos(t_ang(i))];
    arc2(:,i) = R1 * (Rt * xh) * R_arc; 
end
plot3(arc2(1,:), arc2(2,:), arc2(3,:), 'k-', 'LineWidth', 1.5)

% Punta de flecha robusta
dir2 = arc2(:, end) - arc2(:, end-1);
dir2 = dir2 / norm(dir2);
P_start2 = arc2(:, end) - dir2 * tam_flecha;
quiver3(P_start2(1), P_start2(2), P_start2(3), dir2(1)*tam_flecha, dir2(2)*tam_flecha, dir2(3)*tam_flecha, 0, 'k', 'LineWidth', 1.5, 'MaxHeadSize', 0.8);

pt_txt = arc2(:, round(N_arc/2)) * 1.25;
text(pt_txt(1), pt_txt(2), pt_txt(3), '$\theta$', 'Interpreter','latex','FontSize',18,'HorizontalAlignment','center');

t6 = text(L*x1(1),L*x1(2),L*x1(3),'$\mathbf{x}_1$','Interpreter','latex','Margin',2);
t7 = text(L*y1(1),L*y1(2),L*y1(3),'$\mathbf{y}_1=\mathbf{y}_2$','Interpreter','latex','Margin',2);
t8 = text(L*z1(1),L*z1(2),L*z1(3),'$\mathbf{z}_1$','Interpreter','latex','Margin',2);
t9 = text(L*x2(1),L*x2(2),L*x2(3),'$\mathbf{x}_2$','Interpreter','latex','Margin',2);
t11= text(L*z2(1),L*z2(2),L*z2(3),'$\mathbf{z}_2$','Interpreter','latex','Margin',2);
axis tight; camzoom(1.3);

%% =========================================================
%% 3. Balance (Eje anterior: 2 -> Eje nuevo: b)
%% =========================================================
nexttile
hold on; axis equal; axis off
view(35,20)
R3 = R2*Rx;
V3 = (R3*V0')';
patch('Vertices',V3,'Faces',F, 'FaceColor',[0.5 0.5 0.5], 'EdgeColor','none', 'FaceLighting','gouraud');
material dull; camlight headlight; lighting gouraud

% Ejes anteriores (x2, y2, z2 en Negro)
quiver3(0,0,0,L*x2(1),L*x2(2),L*x2(3),'k','LineWidth',1,'MaxHeadSize',0.1)
quiver3(0,0,0,L*y2(1),L*y2(2),L*y2(3),'k','LineWidth',1,'MaxHeadSize',0.1)
quiver3(0,0,0,L*z2(1),L*z2(2),L*z2(3),'k','LineWidth',1,'MaxHeadSize',0.1)

% Ejes nuevos (Color)
xb = x2; yb = R3*yh; zb = R3*zh;
quiver3(0,0,0,L*xb(1),L*xb(2),L*xb(3),'Color',[0.6 0 0],'LineWidth',1.2,'MaxHeadSize',0.1)
quiver3(0,0,0,L*yb(1),L*yb(2),L*yb(3),'Color',[0 0.5 0],'LineWidth',1.2,'MaxHeadSize',0.1)
quiver3(0,0,0,L*zb(1),L*zb(2),L*zb(3),'Color',[0 0 0.5],'LineWidth',1.2,'MaxHeadSize',0.1)

% Dibujo del Arco
t_ang = linspace(0, phi, N_arc);
arc3 = zeros(3, N_arc);
for i = 1:N_arc
    Rt = [1 0 0; 0 cos(t_ang(i)) sin(t_ang(i)); 0 -sin(t_ang(i)) cos(t_ang(i))];
    arc3(:,i) = R2 * (Rt * yh) * R_arc; 
end
plot3(arc3(1,:), arc3(2,:), arc3(3,:), 'k-', 'LineWidth', 1.5)

% Punta de flecha robusta
dir3 = arc3(:, end) - arc3(:, end-1);
dir3 = dir3 / norm(dir3);
P_start3 = arc3(:, end) - dir3 * tam_flecha;
quiver3(P_start3(1), P_start3(2), P_start3(3), dir3(1)*tam_flecha, dir3(2)*tam_flecha, dir3(3)*tam_flecha, 0, 'k', 'LineWidth', 1.5, 'MaxHeadSize', 0.8);

pt_txt = arc3(:, round(N_arc/2)) * 1.25;
text(pt_txt(1), pt_txt(2), pt_txt(3), '$\phi$', 'Interpreter','latex','FontSize',18,'HorizontalAlignment','center');

t12 = text(L*x2(1),L*x2(2),L*x2(3),'$\mathbf{x}_2=\mathbf{x}_b$','Interpreter','latex','Margin',2);
t13 = text(L*y2(1),L*y2(2),L*y2(3),'$\mathbf{y}_2$','Interpreter','latex','Margin',2);
t14 = text(L*z2(1),L*z2(2),L*z2(3),'$\mathbf{z}_2$','Interpreter','latex','Margin',2);
t16 = text(L*yb(1),L*yb(2),L*yb(3),'$\mathbf{y}_b$','Interpreter','latex','Margin',2);
t17 = text(L*zb(1),L*zb(2),L*zb(3),'$\mathbf{z}_b$','Interpreter','latex','Margin',2);
axis tight; camzoom(1.3);

%% =========================================================
%% Exportación a EPS (Descomenta para guardar)
%% =========================================================
% set(gcf, 'Renderer', 'painters');
% exportgraphics(gcf, 'angulos_euler.eps', 'ContentType', 'vector', 'BackgroundColor', 'none');