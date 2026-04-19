clear all; clc; close all;

%% =======================
% LECTURA DE DATOS (2026)
%% =======================
values = xlsread('Curvas_Medidas_Motor_2026.xls');

tt  = values(:,1);
W   = values(:,2);
Ia  = values(:,3);
Vin = values(:,4);
TL_ = values(:,5);

%% =======================
% GRAFICAS EXPERIMENTALES
%% =======================
figure(1)

subplot(4,1,1); plot(tt,W,'b'); title('Velocidad angular'); grid on;
subplot(4,1,2); plot(tt,Vin,'b'); title('Tensión'); grid on;
subplot(4,1,3); plot(tt,Ia,'b'); title('Corriente'); grid on;
subplot(4,1,4); plot(tt,TL_,'b'); title('Torque'); grid on;

%% =======================
% SEÑALES DE SIMULACION
%% =======================
Ts = 1e-4;
tF = tt(end);
t  = 0:Ts:tF;

E = max(Vin);

um = zeros(size(t));
TL = zeros(size(t));

% detectar escalón real
idx_step = find(Vin > 0.1*E,1);
ret = tt(idx_step);

TLmax = max(TL_);

for i=1:length(t)
    if t(i) >= ret
        um(i) = E;
    end
    
    if ((0.1869<=t(i) && t(i)<0.3372) || t(i)>=0.4872)
        TL(i) = TLmax;
    end
end

%% =======================
% CHEN VELOCIDAD (ROBUSTO)
%% =======================
K_W = max(W)/E;
Wn  = W/max(W);

t63 = tt(find(Wn>=0.63,1));

T1_W = 1.5*(t63-ret);
T2_W = 0.5*(t63-ret);

if isnan(T1_W) || isnan(T2_W) || T1_W<=0
    T1_W = 0.01;
    T2_W = 0.02;
end

G_W = tf(K_W,[T1_W*T2_W T1_W+T2_W 1]);

[y_G_W,t_G_W] = lsim(G_W,um,t);

figure(2)
plot(tt,W,'k'); hold on;
plot(t_G_W,y_G_W,'r');
legend('Real','Modelo Chen');
title('Velocidad');

%% =======================
% CHEN CORRIENTE (ROBUSTO)
%% =======================
K_ia = max(Ia)/E;
Ian  = Ia/max(Ia);

t63_i = tt(find(Ian>=0.63,1));

T1_ia = 1.5*(t63_i-ret);
T2_ia = 0.5*(t63_i-ret);

if isnan(T1_ia) || isnan(T2_ia) || T1_ia<=0
    T1_ia = 0.01;
    T2_ia = 0.02;
end

G_ia = tf(K_ia,[T1_ia*T2_ia T1_ia+T2_ia 1]);

[y_G_ia,t_G_ia] = lsim(G_ia,um,t);

figure(3)
plot(tt,Ia,'k'); hold on;
plot(t_G_ia,y_G_ia,'r');
legend('Real','Modelo Chen');
title('Corriente');

%% =======================
% PARAMETROS MOTOR (CORREGIDOS)
%% =======================
Ra = max(Vin)/max(Ia);   % resistencia realista
Km = 1/K_W;
Ki = Km;

J  = 1e-4;
La = 1e-3;
Bm = 1e-5;

%% =======================
% ESPACIO DE ESTADOS
%% =======================
A = [-Ra/La  -Km/La  0;
      Ki/J   -Bm/J   0;
      0       1      0];

B = [1/La  0;
     0    -1/J;
     0     0];

C = [0 1 0];

x = [0;0;0];

wr = zeros(size(t));
ia = zeros(size(t));
theta = zeros(size(t));

%% =======================
% SIMULACION
%% =======================
for ii=1:length(t)-1
    xdot = A*x + B*[um(ii); TL(ii)];
    x = x + xdot*Ts;
    
    ia(ii+1) = x(1);
    wr(ii+1) = x(2);
    theta(ii+1) = x(3);
end

%% =======================
% GRAFICAS FINALES
%% =======================
figure(4)

subplot(4,1,1)
plot(tt,W,'g'); hold on;
plot(t,wr,'k');
title('Velocidad');
legend('Real','Modelo')

subplot(4,1,2)
plot(t,um,'r');
title('Tensión')

subplot(4,1,3)
plot(tt,Ia,'g'); hold on;
plot(t,ia,'k');
title('Corriente')
legend('Real','Modelo')

subplot(4,1,4)
plot(tt,TL_,'g'); hold on;
plot(t,TL,'k');
title('Torque')
legend('Real','Modelo')

