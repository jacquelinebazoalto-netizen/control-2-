% Sistemas de Control II - Motor CC
% Control por Variables de Estado + Observador
% Incisos a, b y c

clc;
clear;
close all;

%% LECTURA DE DATOS

data = readmatrix("Curvas_Medidas_Motor_2026 .xls");

t  = data(:,1);
w  = data(:,2);
ia = data(:,3);
Va = data(:,4);
TL = data(:,5);

%% MODELADO DEL MOTOR

StepAmplitude = 10;

k = w(end)/StepAmplitude

Imax = max(abs(ia))

Ra = 10/Imax

K_tl = (30.33 - 66.66)/20

Km = 1/k

Ki = -Ra/(K_tl*Km)

Bm = 0;

J = (0.0945*Ki*Km)/Ra

La = (0.0002148*Ki*Km)/J

%% ESPACIO DE ESTADOS CONTINUO

Ac = [ -Ra/La   -Km/La   0;
        Ki/J    -Bm/J    0;
        0        1       0];

Bc = [1/La   0;
      0    -1/J;
      0      0];

Cc = [0 1 0;
      0 0 1];

Dc = [0 0;
      0 0];

sysCont = ss(Ac,Bc,Cc,Dc)

%% POLOS Y TIEMPO DE MUESTREO

val = sort(real(eig(Ac)))

% Polo mas rapido
poloRapido = val(1)

% Polo mas lento
poloLento = val(end-1)

% Dinamica rapida
tR = abs(log(0.95)/poloRapido)

% Integracion Euler
tInt = tR/4

% Tiempo de muestreo
Ts = tInt

fprintf('\nTiempo de muestreo Ts = %f s\n',Ts);

%% SISTEMA DISCRETO

sysDisc = c2d(sysCont,Ts,'zoh');

Ad = sysDisc.a;

Bd = sysDisc.b;

Bd_aux = Bd(:,1);

Cd = sysDisc.c;

Dd = sysDisc.d;

%% CONTROLABILIDAD

Mc = [Bd_aux Ad*Bd_aux Ad^2*Bd_aux];

fprintf('\nRango de controlabilidad = %d\n',rank(Mc));

%% SISTEMA AMPLIADO

Aa = [Ad zeros(3,1);
     -Cd(2,:)*Ad 1];

Ba = [Bd_aux;
     -Cd(2,:)*Bd_aux];

%% CONTROLADOR LQR

Q = diag([100 1 0.5 1]);

R = 4e4;

[Klqr,~,~] = dlqr(Aa,Ba,Q,R);

K = Klqr(1:3);

KI = -Klqr(4);

%% OBSERVADOR

Ao = Ad';

Bo = Cd';

Qo = diag([100 0.1 0.1]);

Ro = diag([1 1]);

Ko = (dlqr(Ao,Bo,Qo,Ro))';

%% SIMULACION

dT = Ts;

Tf = 30;

p_max = floor(Tf/dT);

tt = 0:dT:p_max*dT;

deadZone = 0.5;

%% VARIABLES

Ia = zeros(1,p_max+1);

Wr = zeros(1,p_max+1);

Theta = zeros(1,p_max+1);

Ia_ob = zeros(1,p_max+1);

u = zeros(1,p_max+1);

ref = zeros(1,p_max+1);

TL_v = zeros(1,p_max+1);

%% REFERENCIA ANGULAR

thetaRef = pi/2;

tSwitch = 15;

counter = 0;

for i = 1:p_max

    counter = counter + dT;

    if(counter > tSwitch)

        thetaRef = -thetaRef;

        counter = 0;

    end

    ref(i) = thetaRef;

end

%% TORQUE DE CARGA

TL_v(tt>=0.7) = 0.12;

%% CONDICIONES INICIALES

x = [0 0 0]';

xob = [0 0 0]';

ei = 0;

%% SIMULACION DEL SISTEMA

for i = 1:p_max

    y = Cd*x;

    y_ob = Cd*xob;

    %% ERROR INTEGRAL

    ei = ei + (ref(i) - y(2));

    %% LEY DE CONTROL

    u(i) = -K*xob + KI*ei;

    %% SATURACION A ±5V

    u(i) = max(min(u(i),5),-5);

    %% ZONA MUERTA

    if(abs(u(i)) < deadZone)

        u(i) = 0;

    else

        u(i) = sign(u(i))*(abs(u(i)) - deadZone);

    end

    %% MOTOR REAL

    Iap = -(Ra/La)*Ia(i) ...
          -(Km/La)*Wr(i) ...
          +(1/La)*u(i);

    Ia(i+1) = Ia(i) + Iap*dT;

    Wrp = (Ki/J)*Ia(i) ...
          -(Bm/J)*Wr(i) ...
          -(1/J)*TL_v(i);

    Wr(i+1) = Wr(i) + Wrp*dT;

    Theta(i+1) = Theta(i) + Wr(i)*dT;

    %% ACTUALIZACION ESTADO REAL

    x = [Ia(i+1) Wr(i+1) Theta(i+1)]';

    %% OBSERVADOR

    xob = Ad*xob + Bd_aux*u(i) + Ko*(y - y_ob);

    %% GUARDAR CORRIENTE ESTIMADA

    Ia_ob(i+1) = xob(1);

end

%% GRAFICO ANGULO

figure

plot(tt,ref,'b','LineWidth',1.5)
hold on

plot(tt,Theta,'r','LineWidth',1.5)

title('Angulo \theta [rad]')

xlabel('Tiempo [s]')

ylabel('\theta [rad]')

yticks([-pi/2 0 pi/2])

yticklabels({'-\pi/2','0','\pi/2'})

legend('Referencia','Theta')

grid on

%% GRAFICO VELOCIDAD ANGULAR

figure

plot(tt,Wr,'r','LineWidth',1.5)

title('Velocidad Angular')

xlabel('Tiempo [s]')

ylabel('\omega [rad/s]')

grid on

%% GRAFICO CORRIENTE REAL VS ESTIMADA

figure

plot(tt,Ia,'b','LineWidth',1.5)

hold on

plot(tt,Ia_ob,'r--','LineWidth',1.5)

title('Corriente Real vs Estimada')

xlabel('Tiempo [s]')

ylabel('Ia [A]')

legend('Ia real','Ia estimada')

grid on

%% GRAFICO ACCION DE CONTROL

figure

plot(tt,u,'k','LineWidth',1.5)

title('Accion de Control')

xlabel('Tiempo [s]')

ylabel('Voltaje [V]')

grid on

%% GRAFICO TORQUE

figure

plot(tt,TL_v,'g','LineWidth',1.5)

title('Torque de Carga')

xlabel('Tiempo [s]')

ylabel('TL [Nm]')

grid on

%% PLANO DE FASE

figure

plot(Theta,Wr,'m','LineWidth',1.5)

title('Plano de Fase')

xlabel('\theta')

ylabel('\omega')

grid on
