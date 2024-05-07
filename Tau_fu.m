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
    block.NumOutputPorts = 1;

    %% Setup functional port properties to dynamically
    %% inherited.
    block.SetPreCompInpPortInfoToDynamic;
    block.SetPreCompOutPortInfoToDynamic;

    block.InputPort(1).Dimensions        = 1;
    block.InputPort(1).DirectFeedthrough = false;
   


    block.OutputPort(1).Dimensions       = 1;
    

    %% Set block sample time to continuous
    block.SampleTimes = [0 0];

    
    block.NumContStates = 1;

    %% Set the block simStateCompliance to default (i.e., same as a built-in block)
    block.SimStateCompliance = 'DefaultSimState';

    %% Register methods
    
    block.RegBlockMethod('InitializeConditions', @InitConditions);
    block.RegBlockMethod('Outputs',   @Output);  
    block.RegBlockMethod('Derivatives', @Derivative);  

    
end
function InitConditions(block)
    
    block.ContStates.Data(1) = 0;  
    
end





function Output(block)
    block.OutputPort(1).Data = block.ContStates.Data(1); %Tau_fu

end

function Derivative(block)
    Fc = 0.0456;
    w = 0.05;
    Fs = 0.0717;
    b = 0.00796;
    k = 1.35;
    
    dPhi = block.InputPort(1).Data;
    
    % Limitar el rango de dPhi para evitar valores extremadamente grandes
    dPhi = max(min(dPhi, 1), -1); % Por ejemplo, limitado entre -10 y 10
    
    % Ajustar la magnitud de la constante multiplicativa
    block.ContStates.Data(1) = b*dPhi + 1 + (k*(Fc + (Fs-Fc)*exp(-1*(dPhi/w))))*tanh(10*dPhi);
end
