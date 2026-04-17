close all; clear; clc;

%%
% Parámetros del sistema
R = 2200; 
L = 500e-3; 
Cap = 10e-6; 

vin = 12;   % amplitud

% Condiciones iniciales
x = [0; 0];   % [Il; Vc]

% Espacio de estados
A = [-R/L  -1/L; 
      1/Cap  0];
B = [1/L; 0];
C = [R 0];

%%
% Función de transferencia (solo para ver polos)
[numG, denG] = ss2tf(A,B,C,0);
G = tf(numG, denG)

poles = roots(denG)

%%
% TIEMPO (🔥 CORREGIDO)
tint = 1e-5;        % paso de integración correcto
tsim = 0.05;        % 50 ms

t = 0:tint:tsim;    % 🔥 vector de tiempo correcto
N = length(t);

% Inicialización
u  = zeros(1,N);
y  = zeros(1,N);
Il = zeros(1,N);
Vc = zeros(1,N);

%%
% Señal cuadrada (cambia cada 10 ms)
Tswitch = 0.01;

for i=1:N-1
    
    % Generación robusta de señal cuadrada
    if sin(2*pi*(1/(2*Tswitch))*t(i)) >= 0
        u(i) = vin;
    else
        u(i) = -vin;
    end
    
    % Dinámica del sistema (Euler)
    xp = A*x + B*u(i);
    x = x + xp*tint;
    
    % Salidas
    y(i)  = C*x;
    Il(i) = x(1);
    Vc(i) = x(2);
end

%%
% GRÁFICAS

figure

subplot(4,1,1)
plot(t,Il,'b','LineWidth',1.2)
title('Corriente i_L')
grid on

subplot(4,1,2)
plot(t,Vc,'r','LineWidth',1.2)
title('Tensión en el capacitor v_C')
grid on

subplot(4,1,3)
stairs(t,u,'m','LineWidth',1.5)   % 🔥 entrada bien cuadrada
title('Entrada u(t)')
grid on

subplot(4,1,4)
plot(t,y,'k','LineWidth',1.2)
title('Salida v_R')
grid on
