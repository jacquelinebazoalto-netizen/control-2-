function [X]=modmotorpruebatita(t_etapa, xant, accion)
Laa= 1e-04; J=1e-04;Ra=23;B=0;Ki= 0.7;Km=0.15;
Va=accion(1);
Tl=accion(2);
h=10e-7;
ia=xant(1);
w= xant(2);
tita=xant(3);

for ii=1:t_etapa/h
  iap=(-Ra/Laa)*ia -(Km/Laa)*w + (1/Laa)*Va;
  wp=(Ki/J)*ia -(B/J)*w - (1/J)*Tl;

  ia=ia+h*iap;
  w = w + h*wp;
  tita=tita+h*w;
end
X=[ia, w, tita];
