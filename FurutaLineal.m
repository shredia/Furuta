function FurutaLineal(block)
% Level-2 MATLAB file S-Function for limited integrator demo.
%   Copyright 1990-2009 The MathWorks, Inc.

setup(block);
end

function setup(block)
    %% Register number of dialog parameters   
    block.NumDialogPrms = 0;

    %% Register number of input and output ports
    block.NumInputPorts  = 1;
    block.NumOutputPorts = 5;

    %% Setup functional port properties to dynamically
    %% inherited.
    block.SetPreCompInpPortInfoToDynamic;
    block.SetPreCompOutPortInfoToDynamic;

    block.InputPort(1).Dimensions        = 1;
    block.InputPort(1).DirectFeedthrough = false;
    
    


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

    K = 1.32;
    R = 9.3;
    L = 43.1;
    



    % Entradas
    
    A = [0,0,1.0000,0,0;0,0,0,1.0000,0;-0.0197,0,0,0,-0.0000; 0 ,0,-0.0306, 0, -0.2158;0.0387,0,0,0,0.3519];
   
    

    % Calcular las derivadas
    x = [block.ContStates.Data(1); block.ContStates.Data(2); block.ContStates.Data(3); block.ContStates.Data(4); block.ContStates.Data(5)];  % 
    U = block.InputPort(1).Data;  % 
    B = [0;0;0;0;1/L];
    dx_dt = A * x + B * U;
    block.Derivatives.Data = dx_dt;
end
