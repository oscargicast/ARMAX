clear all; close all; clc;
line_widht = 1;

%% Toolbox de identificacion mediante comandos Matlab
%% DATA experimental proviene desde el circuito 3 Opams
% OPAMS
data = load('data/data_seno.lvm');
y = data(:, 4);
u = data(:, 6);

subplot(211);
plot(y, 'r'), ylabel('Amplitud')

subplot(212);
plot(u, 'k'), ylabel('Amplitud'), xlabel('N(muestras)')

N = length(u);
disp('# de muestras'), disp(N)

tini = 0;
tfin = 10;
t = linspace(tini, tfin, N);

% tt = data(:,1);


% t = 0:N-1;
% u = ones(1, N);

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

back = data(:,2);

figure; hold on;
plot(t, yt, 'g', 'LineWidth', line_widht);
plot(t, y, 'r', 'LineWidth', line_widht);
% plot(t, back, 'k', 'LineWidth', line_widht);


%% Toolbox de identificacion

%% 1st. Porceso de la data-objeto - iddata DAT = iddata(Y,U,Ts)
Ts = 1/30; 
idata = iddata(y, u, Ts);

%% 2nd Estructura parametrica ARX(na, nb, nx=nc)
% na = 2; nb = 2; nc = 2; nk = 1;
na = 3; nb = 2; nc = 1; nk = 1;

% th = arx(idata, [na, nb, nc]);
th = armax(idata, [na, nb, nc nk]);
% En versiones anteriores se tiena a 'q' en vez de 'z'
% B numerador, A denominador
% FPE(funcion de prediccion de error)
th

%% 3rd discreta - funcion de transerencia D(z)
D = tf(th.b, th.a, Ts)
De = tf(th.c, th.a, Ts)
% cmd: d2c

%% 4th Funcion de transferencia G(s)
Gs = d2c(D, 'zoh');
Ge = d2c(De, 'zoh');
disp('funcion de trans'), Gs

% % Son diferentes Gs y G: th2th es obsoleta
% [n, d] = th2tf(th); % de formato theta a tf
% % th2th es obsoleta, se debe usar TFDATA
% G = tf(n, d);
% disp('funcion de trans'), G

[n, d] = tfdata(D, 'v'); 
Gs = d2c(D, 'zoh');

% white noise
e = wgn(1,N,0);

yc = lsim(Gs, u, t);
ye = lsim(Ge, 0.001*e, t);
plot(t, yc+ye, 'b', 'LineWidth', line_widht);

legend('y_{teo}', 'y_{exp}', 'y_{iden}', 4);



%% Indentificaision usando GUIDE
% ident




