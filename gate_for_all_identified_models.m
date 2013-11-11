clear all; close all; clc;
line_widht = 1.5;
%% Input ARMAX parameters
nn = input('ARMAX(na,nb,nc,nk): ');
if length(nn)~=4
    error('Error, input must be an 1x4 array!, ejmp: [na, nb, nc, nk]')
    return 
end
%% Input data
data_type = 'gate';
validatestring(data_type, {'gate', 'step', 'seno', 'rampa'})
file = strcat('data/data_', data_type, '.lvm');
data = load(file);

%% Toolbox de identificacion mediante comandos Matlab
%% DATA experimental proviene desde el circuito 3 Opams
% OPAMS
u = data(:, 6); % GATE
y = data(:, 4);
back = data(:,2);

N = length(u);
disp('# de muestras'), disp(N);

tini = 0;
tfin = 10;
t = linspace(tini, tfin, N);

%% Analisis teorico
% Parametros
R1 = 22e3;
R2 = 47e3;
R3 = 68e3;
R4 = 22e3;
R5 = 10e3;
R6 = 2.2e3;
R7 = 1e3;
R8 = 1e3;

C1 = 4.7e-6;
C2 = 10e-6;
C3 = 1e-6;

A = R6/(R5+R6);
B = R3*R5*C3/(R5+R6);

m1 = A*(1/R2 + 1/R4) - 1/R4;
m2 = -B*(1/R2 + 1/R4) - R5*C3/(R5+R6) + C2*A;
m3 = B*C2;

% P con C.I == 0
Denom = conv([C1*R1 1], [-m3 m2 m1]);
f = Denom(1);
Denom = Denom/f;

Numer = -R8/(R2*R7);
Numer = Numer/f;

P = tf(Numer, Denom);

yt = lsim(P, u, t);


% teorica
figure; hold on;
plot(t, yt, 'y--', 'LineWidth', line_widht);
% experimental
plot(t, y, 'c--', 'LineWidth', line_widht);

%% Toolbox de identificacion

%% 2nd Estructura parametrica ARMAX(na, nb, nc, nx)
% nn = [
%     2 2 1 1;
% ];
colors = ['k', 'g', 'b', 'r'];

data_type = {'gate', 'step', 'seno', 'rampa'};
for i=1:length(data_type)
fprintf('\n\n\t\t\t\t\t ARMAX(%d, %d, %d, %d)', nn(1,1), nn(1,2), nn(1,3), nn(1,4)); 
fprintf('\n==========================================================='); 

type = data_type{i};
type
validatestring(type , {'gate', 'step', 'seno', 'rampa'})
file = strcat('data/data_', type, '.lvm');
data = load(file);
y = data(:, 4);
uu = data(:, 6);

%% 1st. Porceso de la data-objeto - iddata DAT = iddata(Y,U,Ts)
Ts = 0.03; 
idata = iddata(y, uu, Ts);
    
th = armax(idata, [nn(1,1), nn(1,2), nn(1,3), nn(1,4)]);
% B numerador, A denominador
% FPE(funcion de prediccion de error)
% th

%% 3rd discreta - funcion de transerencia D(z)
D = tf(th.b, th.a, Ts);
D
% De = tf(th.c, th.a, Ts);
% cmd: d2c
th

%% 4th Funcion de transferencia G(s)
% Gs = d2c(D, 'zoh');
% Ge = d2c(De, 'zoh');
% disp('funcion de trans'), Gs;

[n, d] = tfdata(D, 'v'); 
Gs = d2c(D, 'zoh');
Gs

yc = lsim(Gs, u, t);

% armax
plot(t, yc, strcat(colors(i)) , 'LineWidth', line_widht);
% legend(colors(i))
end

ylabel('Amplitude(volts)'), xlabel('t(sec.)');
legend( ...
'y_{teo}', ...
'y_{exp}', ...
strcat('identified model_{', data_type{1},'}'), ...
strcat('identified model_{', data_type{2},'}'), ...
strcat('identified model_{', data_type{3},'}'), ...
strcat('identified model_{', data_type{4},'}') );
% title('Validación de modelos indentificados usando ARMAX %d', 2);

str = sprintf('Validación de modelos indentificados usando ARMAX(%d,%d,%d,%d)', ...
nn(1,1), nn(1,2), nn(1,3), nn(1,4));
title(str);


