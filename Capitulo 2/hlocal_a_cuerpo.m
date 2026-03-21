%% Grado en Ingeniería Aeroespacial - UCLM
% TFG - Generación y análisis de trayectorias de vuelo basadas en
% navegación inercial para misiles
% Alumno - Alejandro Burgos Alemany
% Tutor - Miguel Ánguel Gömez Lopez

clear; clc; close all;

%% Estilo
set(groot,'defaultAxesFontName','Times New Roman')
set(groot,'defaultTextFontName','Times New Roman')
set(groot,'defaultAxesFontSize',11)
set(groot,'defaultAxesLineWidth',0.8)

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

%% =========================================================
%% Figura
%% =========================================================
figure('Color','w','Position',[150 200 1400 500])
tiledlayout(1,3,'Padding','compact','TileSpacing','compact')

%% =========================================================
%% Guiñada
%% =========================================================
nexttile
hold on; axis equal; axis off
view(35,20)

R1 = Rz;
V1 = (R1*V0')';

patch('Vertices',V1,'Faces',F,...
      'FaceColor',[0.5 0.5 0.5],...
      'EdgeColor','none',...
      'FaceLighting','gouraud');
material dull
camlight headlight
lighting gouraud

quiver3(0,0,0,L*xh(1),L*xh(2),L*xh(3),'k','LineWidth',1,'MaxHeadSize',0.1)
quiver3(0,0,0,L*yh(1),L*yh(2),L*yh(3),'k','LineWidth',1,'MaxHeadSize',0.1)
quiver3(0,0,0,L*zh(1),L*zh(2),L*zh(3),'k','LineWidth',1,'MaxHeadSize',0.1)

x1 = R1*xh; y1 = R1*yh; z1 = zh;

quiver3(0,0,0,L*x1(1),L*x1(2),L*x1(3),'Color',[0.6 0 0],'LineWidth',1,'MaxHeadSize',0.1)
quiver3(0,0,0,L*y1(1),L*y1(2),L*y1(3),'Color',[0 0.5 0],'LineWidth',1,'MaxHeadSize',0.1)
quiver3(0,0,0,L*z1(1),L*z1(2),L*z1(3),'Color',[0 0 0.6],'LineWidth',1,'MaxHeadSize',0.1)


% Textos mejorados
t1 = text(L*xh(1),L*xh(2),L*xh(3),'$\mathbf{x}_h$',...
    'Interpreter','latex','BackgroundColor','w','Margin',2);
t2 = text(L*yh(1),L*yh(2),L*yh(3),'$\mathbf{y}_h$',...
    'Interpreter','latex','BackgroundColor','w','Margin',2);
t3 = text(L*zh(1),L*zh(2),L*zh(3),'$\mathbf{z}_h=\mathbf{z}_1$',...
    'Interpreter','latex','BackgroundColor','w','Margin',2);

t4 = text(L*x1(1),L*x1(2),L*x1(3),'$\mathbf{x}_1$',...
    'Interpreter','latex','BackgroundColor','w','Margin',2);
t5 = text(L*y1(1),L*y1(2),L*y1(3),'$\mathbf{y}_1$',...
    'Interpreter','latex','BackgroundColor','w','Margin',2);

uistack([t1 t2 t3 t4 t5],'top')

axis tight
%% =========================================================
%% Asiento
%% =========================================================
nexttile
hold on; axis equal; axis off
view(35,20)

R2 = R1*Ry;
V2 = (R2*V0')';

patch('Vertices',V2,'Faces',F,...
      'FaceColor',[0.5 0.5 0.5],...
      'EdgeColor','none',...
      'FaceLighting','gouraud');
material dull
camlight headlight
lighting gouraud

quiver3(0,0,0,L*xh(1),L*xh(2),L*xh(3),'k','LineWidth',1,'MaxHeadSize',0.1)
quiver3(0,0,0,L*yh(1),L*yh(2),L*yh(3),'k','LineWidth',1,'MaxHeadSize',0.1)
quiver3(0,0,0,L*zh(1),L*zh(2),L*zh(3),'k','LineWidth',1,'MaxHeadSize',0.1)

x2 = R2*xh; y2 = y1; z2 = R2*zh;

quiver3(0,0,0,L*x2(1),L*x2(2),L*x2(3),'Color',[0.6 0 0],'LineWidth',1,'MaxHeadSize',0.1)
quiver3(0,0,0,L*y2(1),L*y2(2),L*y2(3),'Color',[0 0.5 0],'LineWidth',1,'MaxHeadSize',0.1)
quiver3(0,0,0,L*z2(1),L*z2(2),L*z2(3),'Color',[0 0 0.6],'LineWidth',1,'MaxHeadSize',0.1)

t6 = text(L*xh(1),L*xh(2),L*xh(3),'$\mathbf{x}_h$',...
    'Interpreter','latex','BackgroundColor','w','Margin',2);
t7 = text(L*yh(1),L*yh(2),L*yh(3),'$\mathbf{y}_h$',...
    'Interpreter','latex','BackgroundColor','w','Margin',2);
t8 = text(L*zh(1),L*zh(2),L*zh(3),'$\mathbf{z}_h$',...
    'Interpreter','latex','BackgroundColor','w','Margin',2);

t9  = text(L*x2(1),L*x2(2),L*x2(3),'$\mathbf{x}_2$',...
    'Interpreter','latex','BackgroundColor','w','Margin',2);
t10 = text(L*y2(1),L*y2(2),L*y2(3),'$\mathbf{y}_2=\mathbf{y}_1$',...
    'Interpreter','latex','BackgroundColor','w','Margin',2);
t11 = text(L*z2(1),L*z2(2),L*z2(3),'$\mathbf{z}_2$',...
    'Interpreter','latex','BackgroundColor','w','Margin',2);

uistack([t6 t7 t8 t9 t10 t11],'top')

axis tight
%% =========================================================
%% Balance
%% =========================================================
nexttile
hold on; axis equal; axis off
view(35,20)

R3 = R2*Rx;
V3 = (R3*V0')';

patch('Vertices',V3,'Faces',F,...
      'FaceColor',[0.5 0.5 0.5],...
      'EdgeColor','none',...
      'FaceLighting','gouraud');
material dull
camlight headlight
lighting gouraud

quiver3(0,0,0,L*xh(1),L*xh(2),L*xh(3),'k','LineWidth',1,'MaxHeadSize',0.1)
quiver3(0,0,0,L*yh(1),L*yh(2),L*yh(3),'k','LineWidth',1,'MaxHeadSize',0.1)
quiver3(0,0,0,L*zh(1),L*zh(2),L*zh(3),'k','LineWidth',1,'MaxHeadSize',0.1)

xb = x2; yb = R3*yh; zb = R3*zh;

quiver3(0,0,0,L*xb(1),L*xb(2),L*xb(3),'Color',[0.6 0 0],'LineWidth',1,'MaxHeadSize',0.1)
quiver3(0,0,0,L*yb(1),L*yb(2),L*yb(3),'Color',[0 0.5 0],'LineWidth',1,'MaxHeadSize',0.1)
quiver3(0,0,0,L*zb(1),L*zb(2),L*zb(3),'Color',[0 0 0.5],'LineWidth',1,'MaxHeadSize',0.1)

t12 = text(L*xh(1),L*xh(2),L*xh(3),'$\mathbf{x}_h$',...
    'Interpreter','latex','BackgroundColor','w','Margin',2);
t13 = text(L*yh(1),L*yh(2),L*yh(3),'$\mathbf{y}_h$',...
    'Interpreter','latex','BackgroundColor','w','Margin',2);
t14 = text(L*zh(1),L*zh(2),L*zh(3),'$\mathbf{z}_h$',...
    'Interpreter','latex','BackgroundColor','w','Margin',2);

t15 = text(L*xb(1),L*xb(2),L*xb(3),'$\mathbf{x}_b=\mathbf{x}_2$',...
    'Interpreter','latex','BackgroundColor','w','Margin',2);
t16 = text(L*yb(1),L*yb(2),L*yb(3),'$\mathbf{y}_b$',...
    'Interpreter','latex','BackgroundColor','w','Margin',2);
t17 = text(L*zb(1),L*zb(2),L*zb(3),'$\mathbf{z}_b$',...
    'Interpreter','latex','BackgroundColor','w','Margin',2);

uistack([t12 t13 t14 t15 t16 t17],'top')

axis tight