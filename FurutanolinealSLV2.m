function FurutanolinealSLV2(block)
% Nivel-2 archivo de función S de MATLAB para demostración de integrador limitado.

%   Copyright 1990-2009 The MathWorks, Inc.

setup(block);

end

function setup(block)

  %% Registrar el número de parámetros de diálogo
  block.NumDialogPrms = 0;

  %% Registrar el número de puertos de entrada y salida
  block.NumInputPorts  = 1;
  block.NumOutputPorts = 5;

  %% Configurar propiedades de puerto funcional para heredar dinámicamente
  block.SetPreCompInpPortInfoToDynamic;
  block.SetPreCompOutPortInfoToDynamic;

  block.InputPort(1).Dimensions        = 1;
  block.InputPort(1).DirectFeedthrough = false;

  
  block.OutputPort(1).Dimensions       = 1;
   block.OutputPort(2).Dimensions       = 1;
    block.OutputPort(3).Dimensions       = 1;
     block.OutputPort(4).Dimensions       = 1;
     block.OutputPort(5).Dimensions       = 1;
  
  %% Establecer el tiempo de muestreo del bloque a continuo
  block.SampleTimes = [0 0];

  %% Configurar Dwork
  block.NumContStates = 5;

  %% Establecer la conformidad del estado de simulación del bloque a predeterminado (es decir, igual que un bloque integrado)
  block.SimStateCompliance = 'DefaultSimState';

  %% Registrar métodos
  block.RegBlockMethod('InitializeConditions', @InitConditions);
  block.RegBlockMethod('Outputs',                 @Output);  
  block.RegBlockMethod('Derivatives',             @Derivative);  

end

function InitConditions(block)
  % Inicializar Dwork
       
    block.ContStates.Data(1) = pi;  % Theta
    block.ContStates.Data(2) = 0;   % Phi
    block.ContStates.Data(3) = 0;   %dTheta
    block.ContStates.Data(4) = 0;   %dPhi
    block.ContStates.Data(5) = 0;   %Corriente
  
end

function Output(block)
  block.OutputPort(1).Data = block.ContStates.Data(1);
  block.OutputPort(2).Data = block.ContStates.Data(2);
  block.OutputPort(3).Data = block.ContStates.Data(3);
  block.OutputPort(4).Data = block.ContStates.Data(4);
  block.OutputPort(5).Data = block.ContStates.Data(5);
end

function Derivative(block)
 % Definición de constantes
    g = 9.81;       % Valor de la gravedad
           
    J = 0.0321;     % Valor de J
    M_2 = 98/1000;       % Valor de M_2
    l_bi = 8.7;     % Valor de l_bi
    C_x = -4/100;     % Valor de C_x
    C_z = 4.4;      % Valor de C_z
    I_x = 4.39e-4;  % Valor de I_x
    I_z = 1.88e-4;  % Valor de I_z
    K = 1.32;
    R = 9.3;
    L = 43.1;
  

  

  % Matriz Qp
  Qp = [block.ContStates.Data(4); block.ContStates.Data(3)];

  % Matriz M(theta)
  M = [M_2*l_bi^2 + 2*C_x*M_2*l_bi + I_x*sin(block.ContStates.Data(1))^2 + I_z + J, C_z*M_2*l_bi*cos(block.ContStates.Data(1)); C_z*M_2*l_bi*cos(block.ContStates.Data(1)), I_x];
  

  % Matriz D(theta, Phi)
   D=[2*I_x*block.ContStates.Data(1)*cos(block.ContStates.Data(1))*sin(block.ContStates.Data(1)), -C_z*M_2*block.ContStates.Data(1)*l_bi*sin(block.ContStates.Data(1)); -I_x*block.ContStates.Data(4)*cos(block.ContStates.Data(1))*sin(block.ContStates.Data(1)), 0];

  % Matriz F(theta)
 
  F = [0; C_z*M_2*g*sin(block.ContStates.Data(1))];

   %Matriz U
   U = block.InputPort(1).Data;
    

   %ksup
    ksup= M\(U -D*Qp -F);
  %Cálculo de corriente
  di = (block.InputPort(1).Data - R*block.ContStates.Data(5) - K*block.ContStates.Data(4))/L;
  % Derivadas
  dx = [block.ContStates.Data(3); block.ContStates.Data(4); ksup(2);ksup(1);di];
  

  % Asignar derivadas al bloque
  block.Derivatives.Data(1) = dx(1);
  block.Derivatives.Data(2) = dx(2);
  block.Derivatives.Data(3) = dx(3);
  block.Derivatives.Data(4) = dx(4);
  block.Derivatives.Data(5) = dx(5);
end
