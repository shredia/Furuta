function [sys,x0,str,ts,simStateCompliance] = ModeloFuruta(t,x,u,flag,Phi,dPhi,Tau,dTau)
% Esta función S-Function modela el sistema del péndulo Furuta.

% Definición de constantes
g = 9.81;       % Valor de la gravedad
Phi = pi;       % Valor de Phi
dPhi = 0;       % Velocidad angular de Phi
Tau = 0;        % Tau
dTau = 0;       % Velocidad angular de Tau
j = 0.0185;     % Valor de J
m_2 = 98;       % Valor de M_2
l_bi = 8.7;     % Valor de l_bi
c_x = -2.3;     % Valor de C_x
c_z = 4.4;      % Valor de C_z
i_x = 4.39e-4;  % Valor de I_x
i_z = 1.88e-4;  % Valor de I_z
b_p = 1;        % Valor de B_p
b_u = 1;        % Valor de B_u
t = 1;
t_fu = 1;
t_fp = 1;

%Ecuacion sistema lineal
d = (j + 2*m_2*l_bi*c_x + m_2*l_bi^2 +i_z)*i_x;
X11 = 0;
X12 = 0;
X13 = m_2*g*c_z*(j+2*m_2*l_bi*c_x+m_2*l_bi^2+i_z);
X14 = m_2^2*c_z^2*l_bi*g/(d);

X21 = 0;
X22 = 0;
X23 = 0;
X24 = 0;

X31 = 1;
X32 = 0;
X33 = (j+2*m_2*l_bi*c_x+m_2*l_bi^2+i_z)*b_p;
X34 = -m_2*l_bi*c_z*b_p/d;

X41 = 0;
X42 = 1;
X43 = -m_2*l_bi*c_z*b_u/d;
X44 = -i_x*b_u/d;

A = [X11 X21 X31 X41;X12 X22 X32 X42;X13 X23 X33 X43;X14 X24 X34 X44];

B1 = 0;
B2 = 0;
B3 = m_2*l_bi*c_z/d;
B4 = i_x/d;
B = [B1;B2;B3;B4];

C = [1 0 0 0];
D = 0;

% Manejo de los diferentes flags
switch flag
  case 0 % Inicialización
    [sys,x0,str,ts,simStateCompliance] = mdlInitializeSizes(A,B,C,D,xi);
  case 1 % Derivadas
    sys = mdlDerivatives(t,x,u,A,B);
  case 2 % Actualización
    sys = mdlUpdate(t,x,u);
  case 3 % Salidas
    sys = mdlOutputs(t,x,u,C,D);
  case 4 % Próximo hit de tiempo variable
    sys = mdlGetTimeOfNextVarHit(t,x,u);
  case 9 % Terminación
    sys = mdlTerminate(t,x,u);
  otherwise % Manejo de errores
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));
end

end

function [sys, x0, str, ts, simStateCompliance] = mdlInitializeSizes(A, B, C, D)
% Inicialización de tamaños y estados iniciales

sizes = simsizes;
sizes.NumContStates = 0; % No hay estados continuos
sizes.NumDiscStates = 0; % No hay estados discretos
sizes.NumOutputs = size(C, 1); % Número de salidas
sizes.NumInputs = size(B, 2); % Número de entradas
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;

sys = simsizes(sizes);

x0 = []; % No hay estados iniciales
str = [];
ts = [0 0]; % Un único tiempo de muestreo

simStateCompliance = 'UnknownSimState';

% Inicializar las matrices del sistema
sys.A = A;
sys.B = B;
sys.C = C;
sys.D = D;
end

function sys=mdlDerivatives(t, x, u, A, B)
% Calculamos las derivadas de los estados del sistema

% Definimos la matriz de estados
X_T = [x(1) - pi; x(2); x(3); x(4)];

% Calculamos las derivadas utilizando las matrices A y B
sys =  A* X_T + B*u;
end

function sys = mdlUpdate(t,x,u)
% Actualización de estados

sys = [];
end

function sys = mdlOutputs(t, x, u, C, D)
% Calculamos las salidas del bloque

% Definimos la matriz de estados
X_T = [x(1) - pi; x(2); x(3); x(4)];

% Calculamos las salidas utilizando las matrices C y D
sys = C * X_T + D * u;
end

function sys = mdlGetTimeOfNextVarHit(t,x,u)
% Determinación del próximo hit de tiempo variable

sampleTime = 1;
sys = t + sampleTime;
end

function sys = mdlTerminate(t,x,u)
% Tareas al finalizar la simulación

sys = [];
end