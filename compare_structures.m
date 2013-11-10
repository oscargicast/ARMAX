clear all; close all; clc;
line_widht = 1.5;

%% Input
data_type = input('signal(gate/step/seno/rampa): ', 's');
validatestring(data_type, {'gate', 'step', 'seno', 'rampa'})
file = strcat('data/data_', data_type, '.lvm');
data = load(file);

%% Toolbox de identificacion mediante comandos Matlab
%% DATA experimental proviene desde el circuito 3 Opams
% OPAMS
y = data(:, 4);
u = data(:, 6);
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
plot(y, 'r', 'LineWidth', line_widht);
plot(yt, 'k--', 'LineWidth', line_widht);
plot(back, 'g--', 'LineWidth', line_widht);
plot(u, 'b', 'LineWidth', line_widht);

ylabel('Amplitude(volts)'), xlabel('N(samples)')
legend('y_{exp}', 'y_{teo}', 'y_{back}', 'u_{exp}')

figure; hold on;
% experimental
plot(t, y, 'r', 'LineWidth', line_widht);

%% Toolbox de identificacion

%% 1st. Porceso de la data-objeto - iddata DAT = iddata(Y,U,Ts)
Ts = 0.03; 
idata = iddata(y, u, Ts);

%% 2nd Estructura parametrica ARMAX(na, nb, nc, nx)
nn = [
    3 1 1 1; 
    2 2 2 2; 
    2 2 1 1;
    1 1 1 1;
];
colors = ['k', 'g', 'b', 'm'];
size_n=size(nn);
for i=1:size_n(1)
fprintf('\n\n\t\t\t\t\t ARMAX(%d, %d, %d, %d)', nn(i,1), nn(i,2), nn(i,3), nn(i,4)); 
fprintf('\n==========================================================='); 
    
th = armax(idata, [nn(i,1), nn(i,2), nn(i,3), nn(i,4)]);
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
plot(t, yc, strcat(colors(i), '--') , 'LineWidth', line_widht);
% legend(colors(i))
end

ylabel('Amplitude(volts)'), xlabel('t(sec.)');
legend( ...
'y_{exp}', ...
arrayfun(@num2str, nn(4,:), 'UniformOutput', true), ...
arrayfun(@num2str, nn(3,:), 'UniformOutput', true), ...
arrayfun(@num2str, nn(2,:), 'UniformOutput', true), ...
arrayfun(@num2str, nn(1,:), 'UniformOutput', true));




