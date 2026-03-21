%% Grado en Ingeniería Aeroespacial - UCLM
% TFG - Generación y análisis de trayectorias de vuelo basadas en
% navegación inercial para misiles
% Alumno - Alejandro Burgos Alemany
% Tutor - Miguel Ánguel Gömez Lopez

clear; clc; close all;
load('AIM9J.mat');

set(groot,'defaultAxesFontName','Times New Roman')
set(groot,'defaultAxesFontSize',11)

%% Centrar misil
CG = mean(V,1);
V  = V - CG;

L = max(range(V))*0.5;
ang = deg2rad(25);
angles = [-ang 0 ang];

view_angle = [35 20];

%% Rotaciones
Rz = @(a)[ cos(a)  sin(a) 0;
          -sin(a)  cos(a) 0;
           0       0      1];

Ry = @(a)[ cos(a)  0  sin(a);
           0       1  0;
          -sin(a)  0  cos(a)];

Rx = @(a)[ 1  0       0;
           0  cos(a) -sin(a);
           0  sin(a)  cos(a)];

colors = {[0.8 0 0], [0.5 0.5 0.5], [0 0.6 0]};
labels = {'-','0','+'};

%% Guiñada

figure('Color','w','Units','centimeters','Position',[3 3 18 6])
tiledlayout(1,3,'Padding','compact','TileSpacing','compact')

for i = 1:3
    nexttile; hold on; axis equal; axis off; view(view_angle)

    R = Rz(angles(i));
    Vrot = (R*V')';

    % misil
    patch('Vertices',Vrot,'Faces',F,...
          'FaceColor',colors{i},...
          'EdgeColor','none',...
          'FaceLighting','gouraud');
    material dull
    lighting gouraud
    camlight headlight

    % eje z (azul)
    plot3([0 0],[0 0],[-L L],...
          'Color',[0 0 0.6],'LineWidth',1)


    axis tight
end

%% Asiento

figure('Color','w','Units','centimeters','Position',[3 3 18 6])
tiledlayout(1,3,'Padding','compact','TileSpacing','compact')

for i = 1:3
    nexttile; hold on; axis equal; axis off; view(view_angle)

    R = Ry(angles(i));
    Vrot = (R*V')';

    patch('Vertices',Vrot,'Faces',F,...
          'FaceColor',colors{i},...
          'EdgeColor','none',...
          'FaceLighting','gouraud');
    material dull
    lighting gouraud
    camlight headlight

    % eje y (verde)
    plot3([0 0],[-L L],[0 0],...
          'Color',[0 0.5 0],'LineWidth',1)


    axis tight
end

%% Balance 

figure('Color','w','Units','centimeters','Position',[3 3 18 6])
tiledlayout(1,3,'Padding','compact','TileSpacing','compact')

for i = 1:3
    nexttile; hold on; axis equal; axis off; view(view_angle)

    R = Rx(angles(i));
    Vrot = (R*V')';

    patch('Vertices',Vrot,'Faces',F,...
          'FaceColor',colors{i},...
          'EdgeColor','none',...
          'FaceLighting','gouraud');
    material dull
    lighting gouraud
    camlight headlight

    % eje x (rojo)
    plot3([-L L],[0 0],[0 0],...
          'Color',[0.6 0 0],'LineWidth',1)

    axis tight
end
