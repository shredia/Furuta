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
    
    block.ContStates.Data(1) = 0;  % Phi
    block.ContStates.Data(2) = pi;   % Theta
    block.ContStates.Data(3) = 0;   %dPhi
    block.ContStates.Data(4) = 0;   %dTheta
    block.ContStates.Data(5) = 0;   %Corriente
end



function Output(block)
    block.OutputPort(1).Data = block.ContStates.Data(1); %Phi
    block.OutputPort(2).Data = block.ContStates.Data(2); %Theta
    block.OutputPort(3).Data = block.ContStates.Data(3); %dPhi
    block.OutputPort(4).Data = block.ContStates.Data(4); %dTheta
    block.OutputPort(4).Data = block.ContStates.Data(5); %Corriente

    
end

function Derivative(block)
    % Definición de constantes

    K = 1.32;
    R = 9.3;
    L = 43.1;
    



    % Entradas
    
    
    A = [0	0	1.00000000000000	0	0;
0	0	0	1.00000000000000	0;
0	-1.12784590916698	0	0	0;
0	-2.21925984550972	0	0	0;
0	0	-0.0306264501160093	0	-0.215777262180974];
    

    % Calcular las derivadas
    x = [block.ContStates.Data(1); block.ContStates.Data(2); block.ContStates.Data(3); block.ContStates.Data(4); block.ContStates.Data(5)];  % 
    U = block.InputPort(1).Data;  % 
    B = [0;
0;
-0.266656901112579;
-0.791264486955069;
0.0232018561484919];
    dx_dt = A * x + B * U;
    block.Derivatives.Data = dx_dt;
end
