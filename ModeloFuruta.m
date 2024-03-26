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

%Ecuacion sistema no-lineal
M11 = j + m_2*l_bi^2 + 2*m_2*l_bi*m_x + m_z + m_x*sind(Phi).^2;
M12 = m_2*l_bi*c_z*cosd(Phi);
M21 = m_2*l_bi*c_z*cosd(Phi);
M22 = i_x;

M = [M11 M12;
     M21 M22];

D11 = 2*i_x*dPhi*sind(Phi)*cosd(Phi);
D12 = -m_2*l_bi*c_z*sind(Phi)*dPhi;
D21 = -dTau*i_x*cosd(Phi)*sind(Phi);
D22 = 0;

D = [D11 D12;
     D21 D22];

F = [0;m_2*g*c_z*sind(Phi)];
U = [t + t_fu;t_fp];




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

function sys = mdlDerivatives(t, x, u, A, B)
% Calculamos las derivadas de los estados del sistema

% Definimos la matriz de estados
X_T = [x(1) - pi; x(2); x(3); x(4)];

% Calculamos las derivadas utilizando las matrices A y B
sys = inv(M)*[U-D]
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