close all
clc

% 1. Cargar la imagen del mapa
mapImage = imread('mapa4.png'); % Reemplaza 'tu_imagen.png' con la ruta a tu imagen

% 2. Preprocesar la imagen (opcional)
mapImageGray = rgb2gray(mapImage); % Convierte la imagen a escala de grises si es necesario
binaryMap = imbinarize(mapImageGray); % Binariza la imagen para obtener un mapa binario
binaryMap = ~binaryMap; % Invierte la imagen si es necesario

% 3. Crear el objeto binaryOccupancyMap con el mapa binario
resolution = 1; % Puedes ajustar la resolución según tu imagen
map = binaryOccupancyMap(binaryMap, resolution);

% Creamos un espacio de estados 2-dimensional
ss = stateSpaceSE2;
% Creamos un validador de estado a partir de SS
sv = validatorOccupancyMap(ss);
% Cargamos el mapa dibujado
sv.Map = map;
% Definimos una distancia de validación
sv.ValidationDistance = 5;
% Hacemos que las dimensiones del SS sean las mismas del mapa
ss.StateBounds = [map.XWorldLimits;map.YWorldLimits; [-pi pi]];
% Puedes ajustar las dimensiones del SS según el mapa creado
% Si el mapa se crea dentro de la función makemap, estas dimensiones pueden ser ajustadas automáticamente
% ss.StateBounds = [map.XWorldLimits; map.YWorldLimits; [-pi pi]];

% mesas = {[90 115 0]. [420 115 0],[750 115],[460 380 0], [730 380 0], [170 300 0], [550 300 0]};


startLocation = [150 450 0];
endLocation =[460 380 0];

%% Ahora para RRTStar
planner_star = plannerRRTStar(ss,sv);
% Seguimos afinando el camino
planner_star.ContinueAfterGoalReached = true;
% Ponemos un límite para no irnos al infinito
planner_star.MaxIterations = 45000;
planner_star.MaxConnectionDistance = 30;
%% 

tic
[pthObj_star,solnInfo_star] = plan(planner_star,startLocation,endLocation);
%[pthObj_star,solnInfo_star] = plan(planner_star,endLocation,startLocation);

% text = ['Tiempo de ejecucion de RRT*: ',num2str(time_rrt)];
disp('Ruta no encontrada');

figure(resolution)
show(map)
hold on
plot(solnInfo_star.TreeData(:,1),solnInfo_star.TreeData(:,2),'.-'); % tree expansion
plot(pthObj_star.States(:,1),pthObj_star.States(:,2),'r-','LineWidth',2) % draw path

x = pthObj_star.States(:,1);
y = pthObj_star.States(:,2);

% Abre un archivo de texto para escritura
fileID = fopen('path4Try1.txt', 'w');

% Escribe las posiciones en el archivo de texto
for i = 1:numel(x)
    fprintf(fileID, '%f\t%f\n', x(i), y(i));
end

% Cierra el archivo
fclose(fileID);
