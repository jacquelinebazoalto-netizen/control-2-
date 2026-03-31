
R= 220;%[ohm]
L= 500e-3;%[Hy]
Cap= 2.2e-6; %[F]
V_e = 12;% [V] voltaje de entrada
%Matrices del espacio
A= [-R/L -1/L ; 1/Cap 0]; % Matriz de estados
B= [1/L ; 0]; %Matriz de entrada
C= [R 0]; %Matriz de salida
D=[0]; %Matriz de transmisión directa

%Punto de operación de V y I
I1(1)=0;
Vc(1)=0;
y(1)=0;
Xop=[0 0]' ;
x=[I1(1) Vc(1)]';

%convierte de espacio de estados a función de transferencia
[numF,denF] = ss2tf(A,B,C,D)

%Función de transferencia del sistema
F=tf(numF,denF)

poles=roots(denF)
Wd1=imag(poles(1))
Wd2= imag(poles(2))
[Wn,zita]=damp(F)
t_d=(2*pi)/Wd1
%time de integración
delta=t_d/100 %
t_l=log(0.05)/(real(poles(1)))
t_sim=3*t_l
step=round(t_sim/delta)
t = linspace(0, t_sim, step);

u=linspace(0,0,step);

ii=0;

for i=1:step-1
    ii = ii + delta; %Variable acumuladora de tiempo -Creación de  u(t)
    if(ii >= 10e-3) %porque conmuta cada 10 milisegundo
        ii=0;
        V_e=V_e*-1; %Cambio el sentido de crecimiento
    end
    u(i)= V_e;
%Aplicación de Euler
    xp=A*(x-Xop)+B*u(i); %representa la funcion derivada de X
    x=x+xp*delta;    %% Obtengo el valor de X a partir de los valores de su derivada
    Y=C*x;           %Almacena el valor actual de la salida

    %Siempre hago referencia al valor siguiente, ya que el primer valor
    % siempre es cero
    y(i+1)=Y(1);    %Hago que el valor siguiente sea el actual
    I1(i+1)=x(1);
    Vc(i+1)=x(2);
end
u(end)=u(end-1);

disp('Simulación completada')

figure(1);
hold on
subplot(3,1,1)
plot(t,u,'blue','LineWidth',1); title('Tension de entrada , u_t')
grid on
subplot(3,1,2)
plot(t,Vc,'red','LineWidth',1); title('Tension en el capacitor, Vc_t')
grid on
subplot(3,1,3);
plot(t,I1,'blue','LineWidth',1); title('Corriente , i_t');
grid on;


