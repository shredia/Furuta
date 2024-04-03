function FurutaLineal(block)
% Level-2 MATLAB file S-Function for limited integrator demo.
%   Copyright 1990-2009 The MathWorks, Inc.

  setup(block);
  
function setup(block)
  
  %% Register number of dialog parameters   
  block.NumDialogPrms = 0;

  %% Register number of input and output ports
  block.NumInputPorts  = 2;
  block.NumOutputPorts = 1;

  %% Setup functional port properties to dynamically
  %% inherited.
  block.SetPreCompInpPortInfoToDynamic;
  block.SetPreCompOutPortInfoToDynamic;
 
  block.InputPort(1).Dimensions        = 1;
  block.InputPort(1).DirectFeedthrough = false;
  block.InputPort(2).Dimensions        = 1;
  block.InputPort(2).DirectFeedthrough = false;
  
  block.OutputPort(1).Dimensions       = 1;
  %block.OutputPort(2).Dimensions       = 1;
  
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

    % Entradas
    Tau_fu = 1;
    Tau_fp = 1;

    u(1) = block.InputPort(1).Data;
    u(2) = block.InputPort(2).Data;
    % Ecuaciones U
    u(1) = Tau + Tau_fu;
    u(2) = Tau_fp;
    % Matriz U
    U = [u(1); u(2)];

    x(1) = block.ContStates.Data(1);
    x(2) = block.ContStates.Data(2);
    x(3) = block.ContStates.Data(3);
    x(4) = block.ContStates.Data(4);

    % Matriz Qp
    Qp = [x(3); x(4)];

    % Ecuaciones de matriz M(theta)
    Mtheta1_1 = J + M_2 * (l_bi)^2 + 2 * M_2 * l_bi * C_x + I_z + I_x * sin(x(1)) * sin(x(1));
    Mtheta2_1 = M_2 * l_bi * C_z * cos(x(1));
    Mtheta1_2 = M_2 * l_bi * C_z * cos(x(1));
    Mtheta2_2 = I_x;

    % matriz M(theta)
    M = [Mtheta1_1, Mtheta2_1; Mtheta1_2, Mtheta2_2];

    % Ecuaciones de matriz D(theta,Phi)
    DthetaPhi1_1 = 2 * I_x * x(3) * sin(x(1)) * cos(x(1));
    DthetaPhi1_2 = -x(4) * I_x * cos(x(1)) * sin(x(1));
    DthetaPhi2_1 = M_2 * l_bi * C_z * sin(x(1)) * x(3);
    DthetaPhi2_2 = 0;

    % matriz D(theta,Phi)
    D = [DthetaPhi1_1, DthetaPhi2_1; DthetaPhi1_2, DthetaPhi2_2];

    % Ecuaciones de matriz F(theta)
    Ftheta1_1 = 0;
    Ftheta1_2 = M_2 * g * C_z * sin(x(1));

    % matriz F(theta)
    F = [Ftheta1_1; Ftheta1_2];

    % Derivadas
    dx = inv(M) * (U - D * Qp - F);

    % Asignar derivadas al bloque
    block.Derivatives.Data(1) = x(3);
    block.Derivatives.Data(2) = x(4);
    block.Derivatives.Data(3) = dx(1);
    block.Derivatives.Data(4) = dx(2);
