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
    
    


    block.OutputPort(1).Dimensions       = [3,1];
    
  
    

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
    block.OutputPort(1).Data = [block.ContStates.Data(1);block.ContStates.Data(2);block.ContStates.Data(5)]; %Salida de estados
  
    
   

    
end

function Derivative(block)
    % Definición de constantes
 
    
    A =[0,	0,	                1.00000000000000,	    0,	                0;
        0,	0,	                0,	                    1.00000000000000,	0;
        0,	1.127393840260797,	0,	                    0,	                40.233158146451224;
        0,	97.320398684038890,	0,	                    0,	                34.380929110916400;
        0,	0,	                -15.357142857142856,	0,	                -144.7619047619047];
    

    % Calcular las derivadas
    x = [block.ContStates.Data(1); block.ContStates.Data(2); block.ContStates.Data(3); block.ContStates.Data(4); block.ContStates.Data(5)];  % 
    U = block.InputPort(1).Data;  % 
    
    B = [0;
         0;
         0;
         0;
         11.904761904761903];

    dx_dt = A*x+B*U;
     

    block.Derivatives.Data = dx_dt;
end
