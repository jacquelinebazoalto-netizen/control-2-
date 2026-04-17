close all; clear; clc;

% --- Parámetros de diseño (Tus valores reales) ---
R = 2200; L = 500e-3; Cap = 10e-6; vin = 12;   

% --- Espacio de estados ---
A = [-R/L -1/L; 1/Cap 0];  
B = [1/L; 0];              
C = [R 0];                 
D = 0;                     


polos = eig(A);
% tR es la constante de tiempo (inversa de la parte real del polo)
tR = 1 / max(abs(real(polos))); 

% Si pones tint = tR/100 y tR es muy chico, 
% el bucle puede tardar mucho. tR/100 es ideal.
tint = tR / 100; 

% --- Configuración de Simulación ---
tsim = 0.05;         
t = 0:tint:tsim;    % Creamos el vector tiempo primero
N = length(t);      % Medimos cuántos puntos hay

% --- Pre-asignación (ESTO ES CLAVE para que aparezca la gráfica) ---
% Creamos los vectores del mismo tamaño que 't'
u = zeros(1, N);
Il = zeros(1, N);
Vcl = zeros(1, N);
y = zeros(1, N);

x = [0; 0]; % Estado inicial
Tswitch = 0.01; 

% --- Bucle de Simulación ---
for i = 1:N-1
    % Entrada cuadrada
    if mod(floor(t(i)/Tswitch), 2) == 0
        u(i) = 12;
    else
        u(i) = -12;
    end
    
    % Euler
    xp = A*x + B*u(i);
    x = x + xp*tint;
    
    % Guardar (usamos i+1 para no pisar el estado inicial)
    Il(i+1) = x(1);
    Vcl(i+1) = x(2);
    y(i+1) = C*x;
end
u(N) = u(N-1); % Completamos el último punto de la entrada

% --- Gráficas ---
figure(1)
subplot(4,1,1); plot(t, Il, 'b', 'LineWidth', 1); title('Corriente (i_L)'); grid on;
subplot(4,1,2); plot(t, Vcl, 'r', 'LineWidth', 1); title('Tensión Capacitor (v_c)'); grid on;
subplot(4,1,3); plot(t, u, 'k'); title('Entrada (u)'); grid on; ylim([-15 15]);
subplot(4,1,4); plot(t, y, 'm', 'LineWidth', 1); title('Salida (v_r)'); grid on;
