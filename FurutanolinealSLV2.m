function FurutanolinealSLV2(block)
% Nivel-2 archivo de función S de MATLAB para demostración de integrador limitado.

%   Copyright 1990-2009 The MathWorks, Inc.

setup(block);

end

function setup(block)

  %% Registrar el número de parámetros de diálogo
  block.NumDialogPrms = 4; % Por ejemplo, cuatro parámetros

  %% Registrar el número de puertos de entrada y salida
  block.NumInputPorts  = 1;
  block.NumOutputPorts = 4;

  %% Configurar propiedades de puerto funcional para heredar dinámicamente
  block.SetPreCompInpPortInfoToDynamic;
  block.SetPreCompOutPortInfoToDynamic;

  block.InputPort(1).Dimensions        = 1;
  block.InputPort(1).DirectFeedthrough = false;

  
  block.OutputPort(1).Dimensions       = 1;
  block.OutputPort(2).Dimensions       = 1;
  block.OutputPort(3).Dimensions       = 1;
  block.OutputPort(4).Dimensions       = 1;


  
    
  % 
 
  
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
    
    block.ContStates.Data(1) = block.DialogPrm(1).Data; % Phi q1
    block.ContStates.Data(2) = block.DialogPrm(2).Data;   % Theta q2
    block.ContStates.Data(3) = block.DialogPrm(3).Data;  %dPhi
    block.ContStates.Data(4) = block.DialogPrm(4).Data;   %dTheta
    
end

function Derivative(block)
 % Definición de constantes
  %   g = 9.81;       % Valor de la gravedad
  % 
  %   J = 0.0321;     % Valor de J
  %   M_2 = 98/1000;       % Valor de M_2
  %   l_bi = 8.7/100;     % Valor de l_bi
  %   C_x = -4/100;     % Valor de C_x
  %   C_z = 4.4/100;      % Valor de C_z
  %   I_x = 4.39e-4;  % Valor de I_x
  %   I_z = 2.24e-4;  % Valor de I_z
  % 
  % 
  % 
  % 
  % 
  % % Matriz Qp [dPhi,dTheta]
  % Qp = [block.ContStates.Data(3); block.ContStates.Data(4)];
  % 
  % % Matriz M(theta) 
  % M = [M_2*l_bi^2 + 2*C_x*M_2*l_bi + I_x*sin(block.ContStates.Data(2))*sin(block.ContStates.Data(2))+ I_z + J, C_z*M_2*l_bi*cos(block.ContStates.Data(2)); C_z*M_2*l_bi*cos(block.ContStates.Data(2)), I_x];
  % 
  % 
  % % Matriz D(theta, Phi)
  %  D=[2*I_x*block.ContStates.Data(4)*cos(block.ContStates.Data(2))*sin(block.ContStates.Data(2)), -C_z*M_2*l_bi*sin(block.ContStates.Data(2))*block.ContStates.Data(4); -I_x*block.ContStates.Data(3)*cos(block.ContStates.Data(2))*sin(block.ContStates.Data(2)), 0];
  % 
  % % Matriz F(theta)
  % 
  % F = [0; C_z*M_2*g*sin(block.ContStates.Data(2))];
  % 
  %  %Matriz U
  %  U = [block.InputPort(1).Data;0];
  % 
  % 
  %  %ksup ksu(1) = d2phi ksu(2) = d2theta
  %   ksup= M\(U -D*Qp -F);
  % 
  % 
  % 
  % % Derivadas [dPhi,dTheta,d2Phi,d2Theta,di]
  % dx = [block.ContStates.Data(3); block.ContStates.Data(4); ksup(1);ksup(2)];
  % 
  % 
  % % Asignar derivadas al bloque
  % block.Derivatives.Data = dx;

  q1 = block.ContStates.Data(1);
  q2 = block.ContStates.Data(2);
  q1p = block.ContStates.Data(3);
  q2p = block.ContStates.Data(4);
  betha = 100;
  tetha1 =  0.0619;
  tetha2 =  0.0149;
  tetha3 =  0.0185;
  tetha4 =  0.0131;
  tetha5 =  0.5076;
  tetha6 =  0.0083;
  tetha7 =  0.0007;
  tetha8 =  0.0188;
  tetha9 =  0.0087;
  
  M11 = tetha1 + tetha2*sin(q2)^2;
  M12 = tetha3*cos(q2);
  M21 = M12;
  M22 = tetha4;

  M = [M11 M12;M21 M22];

  C11 = 0.5*tetha2*q2p*sin(2*q2);
  C12 = -tetha3*q2p*sin(q2) + 0.5*tetha2*q1p*sin(2*q2);
  C21 = -0.5*tetha2*q1p*sin(2*q2);
  C22 = 0;

  C = [C11 C12;C21 C22];
  g1 = 0;
  g2 = -tetha5*sin(q2);

  g = [g1;g2];

  fv1 = tetha6*q1p;
  fv2 = tetha7*q2p;

  fv = [fv1;fv2];

  fc1 = tetha8*tanh(betha*q1p);
  fc2 = tetha9*tanh(betha*q2p);
  
  fc = [fc1;fc2];
    
  Qp = [q1p; q2p];

  U = [block.InputPort(1).Data;0];

  ksup = M\(U - C*Qp - g- fv - fc);

  block.Derivatives.Data = [q1p; q2p; ksup(1);ksup(2)];
end

function Output(block)

    block.OutputPort(1).Data = block.ContStates.Data(1);
    block.OutputPort(2).Data = block.ContStates.Data(2);
    block.OutputPort(3).Data = block.ContStates.Data(3);
    block.OutputPort(4).Data = block.ContStates.Data(4);


        
end