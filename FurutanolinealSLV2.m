function FurutanolinealSLV2(block)
% Nivel-2 archivo de función S de MATLAB para demostración de integrador limitado.

%   Copyright 1990-2009 The MathWorks, Inc.

setup(block);

end

function setup(block)

  %% Registrar el número de parámetros de diálogo
  block.NumDialogPrms = 0;

  %% Registrar el número de puertos de entrada y salida
  block.NumInputPorts  = 3;
  block.NumOutputPorts = 4;

  %% Configurar propiedades de puerto funcional para heredar dinámicamente
  block.SetPreCompInpPortInfoToDynamic;
  block.SetPreCompOutPortInfoToDynamic;

  block.InputPort(1).Dimensions        = 1;
  block.InputPort(1).DirectFeedthrough = false;
  block.InputPort(2).Dimensions        = 1;
  block.InputPort(2).DirectFeedthrough = false;
  block.InputPort(3).Dimensions        = 1;
  block.InputPort(3).DirectFeedthrough = false;

  block.OutputPort(1).Dimensions       = 1;
   block.OutputPort(2).Dimensions       = 1;
    block.OutputPort(3).Dimensions       = 1;
     block.OutputPort(4).Dimensions       = 1;
  
  %% Establecer el tiempo de muestreo del bloque a continuo
  block.SampleTimes = [0 0];

  %% Configurar Dwork
  block.NumContStates = 4;

  %% Establecer la conformidad del estado de simulación del bloque a predeterminado (es decir, igual que un bloque integrado)
  block.SimStateCompliance = 'DefaultSimState';

  %% Registrar métodos
  block.RegBlockMethod('InitializeConditions', @InitConditions);
  block.RegBlockMethod('Outputs',                 @Output);  
  block.RegBlockMethod('Derivatives',             @Derivative);  

end

function InitConditions(block)
  % Inicializar Dwork
    block.ContStates.Data(1) = 0;  % Estado 1
    block.ContStates.Data(2) = pi; % Estado 2
    block.ContStates.Data(3) = 0;  % Estado 3
    block.ContStates.Data(4) = 0;  % Estado 4
  
end

function Output(block)
  block.OutputPort(1).Data = block.ContStates.Data(1);
  block.OutputPort(2).Data = block.ContStates.Data(2);
  block.OutputPort(3).Data = block.ContStates.Data(3);
  block.OutputPort(4).Data = block.ContStates.Data(4);
end

function Derivative(block)
  % Definir las constantes
  g = 9.81;       % Valor de la gravedad
  
  
  J = 0.0185;     % Valor de J
  M_2 = 98;       % Valor de M_2
  l_bi = 8.7;     % Valor de l_bi
  C_x = -2.3;     % Valor de C_x
  C_z = 4.4;      % Valor de C_z
  I_x = 4.39e-4;  % Valor de I_x
  I_z = 1.88e-4;  % Valor de I_z
  B_p = 1;        % Valor de B_p
  B_u = 1;        % Valor de B_u

  % Entradas
  Tau = block.InputPort(1).Data;
  Tau_fu = block.InputPort(2).Data;
  Tau_fp = block.InputPort(3).Data;

  % Ecuaciones de U
 
  
  
  % Matriz U
  U = [Tau + Tau_fu; Tau_fp];

  % Estados
  
  
  
  

  % Matriz Qp
  Qp = [block.ContStates.Data(3); block.ContStates.Data(4)];

  % Matriz M(theta)
  Mtheta1_1 = J + M_2 * (l_bi)^2 + 2 * M_2 * l_bi * C_x + I_z + I_x * sind(block.ContStates.Data(1)) * sin(block.ContStates.Data(1));
  Mtheta2_1 = M_2 * l_bi * C_z * cosd(block.ContStates.Data(1));
  Mtheta1_2 = M_2 * l_bi * C_z * cosd(block.ContStates.Data(1));
  Mtheta2_2 = I_x;
  M = [Mtheta1_1, Mtheta2_1; Mtheta1_2, Mtheta2_2];

  % Matriz D(theta, Phi)
  DthetaPhi1_1 = 2 * I_x * block.ContStates.Data(3) * sind(block.ContStates.Data(1)) * cosd(block.ContStates.Data(1));
  DthetaPhi1_2 = -block.ContStates.Data(4) * I_x * cosd(block.ContStates.Data(1)) * sind(block.ContStates.Data(1));
  DthetaPhi2_1 = M_2 * l_bi * C_z * sind(block.ContStates.Data(1)) * block.ContStates.Data(3);
  DthetaPhi2_2 = 0;
  D = [DthetaPhi1_1, DthetaPhi2_1; DthetaPhi1_2, DthetaPhi2_2];

  % Matriz F(theta)
  Ftheta1_1 = 0;
  Ftheta1_2 = M_2 * g * C_z * sind(block.ContStates.Data(1));
  F = [Ftheta1_1; Ftheta1_2];

  % Derivadas
  dx = [block.ContStates.Data(3); block.ContStates.Data(4); inv(M) * (U - D * Qp - F)];

  % Asignar derivadas al bloque
  block.Derivatives.Data(1) = dx(1);
  block.Derivatives.Data(2) = dx(2);
  block.Derivatives.Data(3) = dx(3);
  block.Derivatives.Data(4) = dx(4);
end
