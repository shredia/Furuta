function FurutaLineal(block)
% Level-2 MATLAB file S-Function for limited integrator demo.
%   Copyright 1990-2009 The MathWorks, Inc.

  setup(block);
  
function setup(block)
  %%xd
  %% Register number of dialog parameters   
  block.NumDialogPrms = 0;

  %% Register number of input and output ports
  block.NumInputPorts  = 3;
  block.NumOutputPorts = 4;

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
  
  
  block.OutputPort(1).Dimensions       = 1;
  block.OutputPort(2).Dimensions       = 1;
  block.OutputPort(3).Dimensions       = 1;
  block.OutputPort(4).Dimensions       = 1;
  
  %% Set block sample time to continuous
  block.SampleTimes = [0 0];
  
  %% Setup Dwork
  block.NumContStates = 4;

  %% Set the block simStateCompliance to default (i.e., same as a built-in block)
  block.SimStateCompliance = 'DefaultSimState';

  %% Register methods
  block.RegBlockMethod('Outputs',   @Output);  
  block.RegBlockMethod('Derivatives', @Derivative);  
  
function Output(block)

  block.OutputPort(1).Data = block.ContStates.Data(1);
  block.OutputPort(2).Data = block.ContStates.Data(2);
  block.OutputPort(3).Data = block.ContStates.Data(3);
  block.OutputPort(4).Data = block.ContStates.Data(4);

function Derivative(block)
    % Definición de constantes
    g = 9.81;       % Valor de la gravedad
    Tau = 0;        % Tau
    dTau = 0;       % Velocidad angular de Tau
    J = 0.0185;     % Valor de J
    M_2 = 98;       % Valor de M_2
    l_bi = 8.7;     % Valor de l_bi
    C_x = -2.3;     % Valor de C_x
    C_z = 4.4;      % Valor de C_z
    I_x = 4.39e-4;  % Valor de I_x
    I_z = 1.88e-4;  % Valor de I_z
    b_p = 1;
    b_u = 1;
    % Entradas
    
    
    d = (J + 2*M_2*l_bi*C_x + M_2*l_bi^2 + I_z)*I_x - M_2^2*l_bi^2*C_z^2;
    B11 = 0;
    B21 = 0;
    B31 = M_2*l_bi*C_z/d;
    B41 = I_x/d;
    
    B12 = 0;
    B22 = 0;
    B32 = 0;
    B42 = 0;
    

 
    % Matriz U
    B = [B11 B12;
         B21 B22;
         B31 B32;
         B41 B42];
    %%XAB A FILA B COLUMNA

    %PRIMERA COLUMNA
    A11 = 0;
    A21 = 0;
    A31 = M_2*g*C_z*(J+2*M_2*l_bi*C_x+M_2*l_bi^2+I_z)/d;
    A41 = M_2^2*C_z^2*l_bi*g/d;
    
    %SEGUNDA COLUMNA
    A12 = 0;
    A22 = 0;
    A32 = 0;
    A42 = 0;

      %TERCERA COLUMNA
    A13 = 1;
    A23 = 0;
    A33 = -1*(J + 2*M_2*l_bi*C_x + M_2*l_bi^2 + I_z)*b_p/d;
    A43 = -1*M_2*l_bi*C_z*b_p/d;

      %CUARTA COLUMNA
    A14 = 0;
    A24 = 1;
    A34 = -1*M_2*l_bi*C_z*b_u/d;
    A44 = -1*I_x*b_u/d;

    A = [A11 A12 A13 A14;
         A21 A22 A23 A24;
         A31 A32 A33 A34;
         A41 A42 A43 A44];
   x1 = block.ContStates.Data(1);
   x2 = block.ContStates.Data(2);
   x3 = block.ContStates.Data(3);
   x4 = block.ContStates.Data(4);
   x = [x1;x2;x3;x4];
   Tau = [block.InputPort(1).Data];
   Tau_fu = [block.InputPort(2).Data];
   Tau_fp = [block.InputPort(3).Data];
    U = [Tau + Tau_fu; Tau_fp];
   dx_dt = A * x + B * U;
   block.Derivatives.Data = dx_dt;
