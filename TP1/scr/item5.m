
clear all; clc; close all;

values = xlsread('Curvas_Medidas_Motor_2026.xls');


t = values(:,1);
Wr = values(:,2);
Ia= values(:,3);
Va= values(:,4);
Tq = values(:,5);
clear opts tbl
%% 
% Gráficas: 

figure; % Creación de la figura sin especificar un número, MATLAB asignará automáticamente
hold on

subplot(4,1,1);
plot(t, Wr, 'g', 'LineWidth', 2);
title('Velocidad angular');
grid on;

subplot(4,1,2);
plot(t, Ia, 'b', 'LineWidth', 2);
title('Corriente de armadura');
grid on;

subplot(4,1,3);
plot(t, Va, 'r', 'LineWidth', 2);
title('Voltaje de entrada'); 
grid on;

subplot(4,1,4);
plot(t,Tq, 'm', 'LineWidth', 2);
title('Torque'); 
grid on;
hold off
%% 
% Aplico Chen para encontrar Wr/Va

% Apply The Second-Order Systems with differents poles method 

% y(t1)= K*(1+ ((T3-T1)/(T1-T2))*exp(-(t1/T1))- (((T3-T2)/(T1-T2))*exp(-t1/T2)));
% y(2*t1)= K*(1+ ((T3-T1)/(T1-T2))*exp(-(2*t1/T1))- (((T3-T2)/(T1-T2))*exp(-(2*t1)/T2)));
% y(3*t1)= K*(1+ ((T3-T1)/(T1-T2))*exp(-(3*t1/T1))- (((T3-T2)/(T1-T2))*exp(-(3*t1)/T2)));

% Obtain maximium value from t (RLC circuit measure).
max_t= max(t);

% Define step amplitude.
A= 10;

% Take time and voltage arrays
% Time array.
t0= t;
%Vc array.
y=Wr;

%Implementación de Método de CHen para G(s)=Wr/Va
% Take three point to apply Chen Method.

%i=155;
i=3;
t_inic= t0(i);
% Define step.
%h= 62;
h=2;
% Obtain y1.
t_t1= t0(i);
y_t1= y(i)/A
% Obtain y2.
t_2t1= t0(i+h)
y_2t1= y(i+h)/A
% Obtain y3.
t_3t1= t0(i+(2*h))
y_3t1= y(i+(2*h))/A

% Normalize gain
% Add abs(y(end)), because 'end' take value= -12V (Alternate input signal).
K= abs(y(end))/(A)

% Calculating k1, k2, k3.
k1= ((y_t1)/K)-1;
k2= ((y_2t1)/K)-1;
k3= ((y_3t1)/K)-1;

% Calculating b, alfa1, alfa2.
b= 4*(k1^(3))*k3- 3*(k1^2)*(k2^2)- 4*(k2^3)+ (k3^2)+ 6*k1*k2*k3;
alfa1= (k1*k2+ k3- sqrt(b))/(2*((k1^2)+ k2));
alfa2= (k1*k2+ k3+ sqrt(b))/(2*((k1^2)+ k2));

% Calculating Beta.
% beta= (2*(k1^3)+ 3*k1*k2+ k3- sqrt(b))/(sqrt(b));
% Alternative
beta= (k1+alfa2)/(alfa1-alfa2);


% Calculo las ctes de tiempo T1, T2 y T3.
% A t_t1 le resto 0.1 seg. del retardo
desp=0.1
T1= -(t_t1-desp)/(log(alfa1))
T2= -(t_t1-desp)/(log(alfa2))
T3= beta *(T1- T2)+ T1

% Build Transfer Function
s= tf('s');
sys= (K*(T3*s+1))/((T1*s+1)*(T2*s+1)); %%sin retardo
sys1= exp(-s*desp)*sys
% [num, den]= tfdata(sys, 'v');

% Plot approximated step response
% Obtain step response with delay.
% [ys, ts]= step(sys*exp(-s*0.01), 'r-', 0.16);
figure;
[ys, ts]= step(A*sys1, 'r-', 1.5);   %aproximada
plot(t, Wr, 'k.-','LineWidth',1.5); %medida 
hold on;
plot(ts, ys, 'r-.');%aproximada
xlim([0, 1]);
ylim([0, 13]);
legend('Measured', 'Approximated');
title('Approximation vs Measurement');
grid on;
%% 
% Aplico Chen para encontrar Wr/TL

%Amplitud máxima del torque
TL= max(Tq)

% Define step de T_L
At= TL;

% Take time and voltage arrays
% Time array.
t0t= t;
%Vc array.
y1=Wr;

%Implementación de Método de CHen para G(s)=Wr/Va
% Take three point to apply Chen Method.

ii=25;
t_inict= t0t(ii);
% Define step.
%h= 62;
ht=2;
% Obtain y1.
t1w= t0t(ii);
y1w= y1(ii)/At
% Obtain y2.
t2w= t0t(ii+ht)
y2w= y1(ii+ht)/At
% Obtain y3.
t3w= t0t(ii+(2*ht))
y3w= y1(ii+(2*ht))/At

% Normalize gain
% Add abs(y(end)), because 'end' take value= -12V (Alternate input signal).
Kt= abs(y1(end))/(At)


% Calculating k1, k2, k3.
k1t= ((y_t1t)/Kt)-1;
k2t= ((y_2t1t)/Kt)-1;
k3t= ((y_3t1t)/Kt)-1;

% Calculating b, alfa1, alfa2.
bt= 4*(k1t^(3))*k3t- 3*(k1t^2)*(k2t^2)- 4*(k2t^3)+ (k3t^2)+ 6*k1t*k2t*k3t;
alfa1t= (k1t*k2t+ k3t- sqrt(bt))/(2*((k1t^2)+ k2t));
alfa2t= (k1t*k2t+ k3t+ sqrt(bt))/(2*((k1t^2)+ k2t));

% Calculating Beta.
% beta= (2*(k1^3)+ 3*k1*k2+ k3- sqrt(b))/(sqrt(b));
% Alternative
betat= (k1t+alfa2t)/(alfa1t-alfa2t);


% Calculo las ctes de tiempo T1, T2 y T3.
% A t_t1 le resto 0.7 seg. del retardo
despt=0.75
T1t= real(-(t_t1t-despt)/(log(alfa1t)))
T2t= real(-(t_t1t-despt)/(log(alfa2t)))
T3t=real(betat *(T1t- T2t)+ T1t)

% Build Transfer Function
s= tf('s');
sy= (K*(T3t*s+1))/((T1t*s+1)*(T2t*s+1)); %%sin retardo
sys2= exp(-s*despt)*sy
% [num, den]= tfdata(sys, 'v');

% Plot approximated step response
% Obtain step response with delay.
% [ys, ts]= step(sys*exp(-s*0.01), 'r-', 0.16);
figure;
[yst, tst]= step(At*sys2, 'r-', 1.5);   %aproximada
plot(t, Wr, 'k.-','LineWidth',1.5); %medida 
hold on;
plot(tst, yst, 'r-.');%aproximada
xlim([0, 1]);
ylim([0, 13]);
legend('Medida', 'Aproximada');
title('Aproximación vs Medición');
grid on;
%% 
%







%% --- PARÁMETROS DEL MOTOR ---
Tl_inf=1e-3;
i_max = max(abs(i))
Ra = 10/i_max
Km = 1/k
%ASUMO B=0
J = 0.0001 
L = 0.0001
%% --- FUNCIÓN W/TL ---
num = [40e3];
den = conv([T1 1],[T2 1]);
sys = tf(num, den);

[y_T, t_T] = lsim(sys, ent_tm, th);

%% --- GRÁFICAS ---
figure; hold on;

subplot(4,1,1);
plot(t_CHEN, y_CHEN - y_T, 'r'); hold on;
plot(t_D, y_D);
title('Velocidad');

subplot(4,1,2);
plot(t_D, i);
title('Corriente');

subplot(4,1,3);
plot(t_D, u); hold on;
plot(th, ent_va, 'r');
title('Voltaje');

subplot(4,1,4);
plot(t_D, Tm); hold on;
plot(th, ent_tm, 'r');
title('Torque');
