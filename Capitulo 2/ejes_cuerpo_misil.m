%% Grado en Ingeniería Aeroespacial - UCLM
% TFG - Generación y análisis de trayectorias de vuelo basadas en
% navegación inercial para misiles
% Alumno - Alejandro Burgos Alemany
% Tutor - Miguel Ánguel Gömez Lopez

clear; clc; close all;

%% Cargar modelo misil
load('AIM9J.mat');  % V, F

CG = mean(V,1);
V  = V - CG; 

%% Generación figura
figure('Color','w','Units','centimeters','Position',[5 5 14 9])
hold on
axis equal
axis off
view(35,20)
L = max(range(V));

%% Exportación modelo AIM 9J
patch('Vertices',V,...
      'Faces',F,...
      'FaceColor',[0.5 0.5 0.5],...
      'EdgeColor','none',...
      'FaceLighting','gouraud');

material dull
lighting gouraud
camlight headlight

%% Representación ejes cuerpo sobre el misil
xb = [-1 0 0];
yb = [ 0 1 0];
zb = [ 0 0 -1];

scale = 0.5*L;
lw = 1.4;

Px = scale*xb;
Py = scale*yb;
Pz = scale*zb;

quiver3(0,0,0,Px(1),Px(2),Px(3),0,...
        'Color',[0.6 0 0],'LineWidth',1,'MaxHeadSize',0.1);

quiver3(0,0,0,Py(1),Py(2),Py(3),0,...
        'Color',[0 0.5 0],'LineWidth',1,'MaxHeadSize',0.1);

quiver3(0,0,0,Pz(1),Pz(2),Pz(3),0,...
        'Color',[0 0 0.6],'LineWidth',1,'MaxHeadSize',0.1);

%% Texto para los ejes cuerpo
offset = 0.04*L;  

tx = text(Px(1)-offset, Px(2), Px(3), ...
    '$\mathbf{x}_b$', ...
    'Interpreter','latex','FontSize',12,...
    'BackgroundColor','w','Margin',2);

ty = text(Py(1), Py(2)+offset, Py(3), ...
    '$\mathbf{y}_b$', ...
    'Interpreter','latex','FontSize',12,...
    'BackgroundColor','w','Margin',2);

tz = text(Pz(1), Pz(2), Pz(3)-offset, ...
    '$\mathbf{z}_b$', ...
    'Interpreter','latex','FontSize',12,...
    'BackgroundColor','w','Margin',2);

uistack([tx ty tz],'top');

%% Establecimiento límites de la figura
axis tight
