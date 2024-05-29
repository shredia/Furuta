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

  block.InputPort(1).Dimensions        = [1,1];
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
    
    block.ContStates.Data(1) = 0;  % Phi
    block.ContStates.Data(2) = pi;   % Theta
    block.ContStates.Data(3) = 0;   %dPhi
    block.ContStates.Data(4) = 0;   %dTheta
    block.ContStates.Data(5) = 0;   %Corriente
end

function Derivative(block)
 % Definición de constantes
    g = 9.81;       % Valor de la gravedad
           
    J = 0.0321;     % Valor de J
    M_2 = 98/1000;       % Valor de M_2
    l_bi = 8.7/100;     % Valor de l_bi
    C_x = -4/100;     % Valor de C_x
    C_z = 4.4/100;      % Valor de C_z
    I_x = 4.39e-4;  % Valor de I_x
    I_z = 2.24e-4;  % Valor de I_z
    K = 1.29;
    R = 12.16;
    L = 84e-3;
  

  

  % Matriz Qp [dPhi,dTheta]
  Qp = [block.ContStates.Data(3); block.ContStates.Data(4)];

  % Matriz M(theta) 
  M = [M_2*l_bi^2 + 2*C_x*M_2*l_bi + I_x*sin(block.ContStates.Data(2))*sin(block.ContStates.Data(2))+ I_z + J, C_z*M_2*l_bi*cos(block.ContStates.Data(2)); C_z*M_2*l_bi*cos(block.ContStates.Data(2)), I_x];
  

  % Matriz D(theta, Phi)
   D=[2*I_x*block.ContStates.Data(4)*cos(block.ContStates.Data(2))*sin(block.ContStates.Data(2)), -C_z*M_2*l_bi*sin(block.ContStates.Data(2))*block.ContStates.Data(4); -I_x*block.ContStates.Data(3)*cos(block.ContStates.Data(2))*sin(block.ContStates.Data(2)), 0];

  % Matriz F(theta)
 
  F = [0; C_z*M_2*g*sin(block.ContStates.Data(2))];

   %Matriz U
   U = [K*block.ContStates.Data(5);0];
    
    
   %ksup ksu(1) = d2phi ksu(2) = d2theta
    ksup= M\(U -D*Qp -F);
  %Cálculo de corriente [di = v-R*i-k*dPhi/L
  di = (block.InputPort(1).Data - R*block.ContStates.Data(5) - K*block.ContStates.Data(3))/L;


  % Derivadas [dPhi,dTheta,d2Phi,d2Theta,di]
  dx = [block.ContStates.Data(3); block.ContStates.Data(4); ksup(1);ksup(2);di];
  

  % Asignar derivadas al bloque
  block.Derivatives.Data = dx;

end

function Output(block)

    block.OutputPort(1).Data = block.ContStates.Data(1);
    block.OutputPort(2).Data = block.ContStates.Data(2);
    block.OutputPort(3).Data = block.ContStates.Data(3);
    block.OutputPort(4).Data = block.ContStates.Data(4);
    block.OutputPort(5).Data = block.ContStates.Data(5);

        
end