function FurutaLineal(block)
% Level-2 MATLAB file S-Function for limited integrator demo.
%   Copyright 1990-2009 The MathWorks, Inc.

setup(block);
end

function setup(block)
    %% Register number of dialog parameters   
    block.NumDialogPrms = 0;

    %% Register number of input and output ports
    block.NumInputPorts  = 4;
    block.NumOutputPorts = 5;

    %% Setup functional port properties to dynamically
    %% inherited.
    block.SetPreCompInpPortInfoToDynamic;
    block.SetPreCompOutPortInfoToDynamic;

    block.InputPort(1).Dimensions        = 1;
    block.InputPort(1).DirectFeedthrough = false;
    block.InputPort(2).Dimensions        = 1;
    block.InputPort(2).DirectFeedthrough = false;
    
    block.InputPort(3).Dimensions        = 1;
    block.InputPort(3).DirectFeedthrough = false;
    
    block.InputPort(4).Dimensions        = 1;
    block.InputPort(4).DirectFeedthrough = false;
    
    


    block.OutputPort(1).Dimensions       = 1;
    block.OutputPort(2).Dimensions       = 1;
    block.OutputPort(3).Dimensions       = 1;
    block.OutputPort(4).Dimensions       = 1;
    block.OutputPort(5).Dimensions       = 1;
    

    %% Set block sample time to continuous
    block.SampleTimes = [0 0];

    
    block.NumContStates = 5;

    %% Set the block simStateCompliance to default (i.e., same as a built-in block)
    block.SimStateCompliance = 'DefaultSimState';

    %% Register methods
    block.RegBlockMethod('InitializeConditions', @InitConditions);
    
    block.RegBlockMethod('Outputs',   @Output);  
    block.RegBlockMethod('Derivatives', @Derivative);  

    
end

function InitConditions(block)
    
    block.ContStates.Data(1) = pi;  % Theta
    block.ContStates.Data(2) = 0;   % Phi
    block.ContStates.Data(3) = 0;   %dTheta
    block.ContStates.Data(4) = 0;   %dPhi
    block.ContStates.Data(5) = 0;   %Corriente
end



function Output(block)
    block.OutputPort(1).Data = block.ContStates.Data(1); %Theta
    block.OutputPort(2).Data = block.ContStates.Data(2); %Phi
    block.OutputPort(3).Data = block.ContStates.Data(3); %dTheta
    block.OutputPort(4).Data = block.ContStates.Data(4); %dPhi
    block.OutputPort(4).Data = block.ContStates.Data(5); %Corriente

    
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
    
  block.ContStates.Data(1) = block.InputPort(2).Data;
  block.ContStates.Data(2) = block.InputPort(3).Data;
  block.ContStates.Data(5) = block.InputPort(4).Data;


    % Entradas
    
    A = [0, 0, 1, 0, 0; 0, 0, 0, 1, 0; 0, (C_z^2*M_2^2*g*l_bi)/(- C_z^2*M_2^2*l_bi^2 + I_x*M_2*l_bi^2 + 2*C_x*I_x*M_2*l_bi + I_x*I_z + I_x*J), 0, 0, I_x/(- C_z^2*M_2^2*l_bi^2 + I_x*M_2*l_bi^2 + 2*C_x*I_x*M_2*l_bi + I_x*I_z + I_x*J); 0, -(C_z*M_2^2*g*l_bi^2 + C_z*I_z*M_2*g + C_z*J*M_2*g + 2*C_x*C_z*M_2^2*g*l_bi)/(- C_z^2*M_2^2*l_bi^2 + I_x*M_2*l_bi^2 + 2*C_x*I_x*M_2*l_bi + I_x*I_z + I_x*J), 0, 0, (C_z*M_2*l_bi)/(- C_z^2*M_2^2*l_bi^2 + I_x*M_2*l_bi^2 + 2*C_x*I_x*M_2*l_bi + I_x*I_z + I_x*J); 0, 0, 0, K/L, -R/L];
   
    

    % Calcular las derivadas
    x = [block.ContStates.Data(1); block.ContStates.Data(2); block.ContStates.Data(3); block.ContStates.Data(4); block.ContStates.Data(5)];  % 
    U = block.InputPort(1).Data;  % 
    B = [0;0;0;0;1];
    dx_dt = A * x + B * U;
    block.Derivatives.Data = dx_dt;
end
