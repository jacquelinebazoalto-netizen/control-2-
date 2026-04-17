clear all; close all; clc;
pkg load control
pkg load signal
pkg load io

tabla=xlsread('Curvas_Medidas_Motor_2026.xlsx');

X=-[0; 0];ii=0;t_etapa=1e-6;titaRef=1;tF=10e-3;
t_D=tabla(:,1); %Tiempo
y_D=tabla(:,2); %Velocidad angular
i=tabla(:,3); %Corriente de armadura
u=tabla(:,4); %Tension
Tl=tabla(:,5); %Torque

t_etapa=1e-4;
th=0:t_etapa:t_D(end);
ret     =       0.035;


ent_va=zeros(size(th));
ent_tm=zeros(size(th));

for ii=1:length(th)

  if th(ii)>ret

  ent_va(ii)=12;
  end

end

for ii=1:length(th)

  if th(ii)>0.1863

  ent_tm(ii)=1e-3;
  end

  if th(ii)>0.3372

  ent_tm(ii)=0;
  end
  if th(ii)>0.4866

  ent_tm(ii)=1e-3;
  end
end

%Constantes del PID
Kp=10;Ki=.05;Kd=.001;color_='b';

Ts=t_etapa;

A1=((2*Kp*Ts)+(Ki*(Ts^2))+(2*Kd))/(2*Ts);
B1=(-2*Kp*Ts+Ki*(Ts^2)-4*Kd)/(2*Ts);
C1=Kd/Ts;

t_D1=diff(t_D);
ii=0;
e=zeros(size(ent_tm));
acc=e;
X=-[0; 0; 0];
uu=0;
for t=1:length(ent_tm)
   ii=ii+1;
   k=ii+2;

  e(k)=titaRef-X(3); %ERROR
  uu=uu+A1*e(k)+B1*e(k-1)+C1*e(k-2); %PID
  X=modmotorpruebatita(t_etapa, X,  [uu,ent_tm(ii)]);

  x1(ii)=X(1);%ia
  x2(ii)=X(2);%w
  x3(ii)=X(3); %tita
  acc(ii)=uu; %entrada tension
  Torque(ii)=ent_tm(ii);
 end
t=th;


figure(1); hold on;
xlabel('Tiempo [Seg.]');
subplot(4,1,1); hold on;
plot(t,x1,'r'); plot(t_D, i); title('Corriente ia');
subplot(4,1,2); hold on;
plot(t,x3,'r'); title('Salida y, \theta_t');
subplot(4,1,3); hold on;
plot(t,acc,'r'); plot(t_D, u); title('Entrada u_t, v_a');
subplot(4,1,4); hold on;
plot(t,Torque,'r'); plot(t_D, Tl); title('Torque');


