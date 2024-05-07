x0 = [0;pi;0;0;0];
u0 = [0;0];
y0 = [0;0;0;0;0];

xf = [2];
uf = [];
yf = [];

[x_op,u_op,y_op] = trim("simulacion_profe_trim",x0,u0,y0,xf,uf,yf)

[A,B,C,D] = linmod("simulacion_profe_trim",x_op,u_op)
