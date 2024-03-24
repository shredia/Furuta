function [sys,x0,str,ts,simStateCompliance] = ModeloFuruta(t,x,u,flag,xi,g,Phi,dPhi,Tau,dTau,J,M_2,l_bi,C_x,C_z,I_x,I_z,B_p,B_u,d)


% The following outlines the general structure of an S-function.
%
switch flag,

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0,
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(xi);

  %%%%%%%%%%%%%%%
  % Derivatives %
  %%%%%%%%%%%%%%%
  case 1,
    sys=mdlDerivatives(t,x,u,la,lp,ma,mp,M,J,g);

  %%%%%%%%%%
  % Update %
  %%%%%%%%%%
  case 2,
    sys=mdlUpdate(t,x,u);

  %%%%%%%%%%%
  % Outputs %
  %%%%%%%%%%%
  case 3,
    sys=mdlOutputs(t,x,u);

  %%%%%%%%%%%%%%%%%%%%%%%
  % GetTimeOfNextVarHit %
  %%%%%%%%%%%%%%%%%%%%%%%
  case 4,
    sys=mdlGetTimeOfNextVarHit(t,x,u);

  %%%%%%%%%%%%%
  % Terminate %
  %%%%%%%%%%%%%
  case 9,
    sys=mdlTerminate(t,x,u);

  %%%%%%%%%%%%%%%%%%%%
  % Unexpected flags %
  %%%%%%%%%%%%%%%%%%%%
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end

% end sfuntmpl

%
%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%
    function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(xi)

    %
    % call simsizes for a sizes structure, fill it in and convert it to a
    % sizes array.
%
% Note that in this example, the values are hard coded.  This is not a
% recommended practice as the characteristics of the block are typically
% defined by the S-function parameters.
%
sizes = simsizes;

sizes.NumContStates  = 4; %Estados continuos
sizes.NumDiscStates  = 0; %Estados discretos
sizes.NumOutputs     = 4; %salidas
sizes.NumInputs      = 1; %entradas
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);

%
% initialize the initial conditions
% condicones iniciales
x0  = xi;

%
% str is always an empty matrix
%
str = [];

%
% initialize the array of sample times
%
ts  = [0 0];

% Specify the block simStateCompliance. The allowed values are:
%    'UnknownSimState', < The default setting; warn and assume DefaultSimState
%    'DefaultSimState', < Same sim state as a built-in block
%    'HasNoSimState',   < No sim state
%    'DisallowSimState' < Error out when saving or restoring the model sim state
simStateCompliance = 'UnknownSimState';
end
% end mdlInitializeSizes

%
%=============================================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%=============================================================================
%
function sys=mdlDerivatives(t,x,u,g,Phi,dPhi,Tau,dTau,J,M_2,l_bi,C_x,C_z,I_x,I_z,B_p,B_u,d)
%Definimos la matriz A y sus componentes, para realizar el cálculo de la
%linealización
a_13 = M_2*g*C_z(J+2*M_2*l_bi*C_x+M_2*l_bi^2+I_z)/d;
a_33 = -(J+2*M_2*l_bi*C_x+M_2*l_bi^2+I_z)*B_p/d;
a_34 = -(M_2*l_bi*C_z*B_u)/d;
a_14 = (M_2^2)*(C_z)^2*l_bi*g/d;
a_34 = -M_2*l_bi*C_z*B_p/d;
a_44 = -I_x*B_u/d;
A = [0      0       1       0;
     0      0       0       1;
     a_13   0       a_33    a_34;
     a_14   0       a_34    a_44];
%falta agregar lo que es tau o la entrada para que quede de la manera 
% dx = Ax+Btau




end
% end mdlDerivatives

%
%=============================================================================
% mdlUpdate
% Handle discrete state updates, sample time hits, and major time step
% requirements.
%=============================================================================
%
function sys=mdlUpdate(t,x,u)

sys = [];
end
% end mdlUpdate

%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
function sys=mdlOutputs(t,x,u)
%salidas, primera angulo del brazo, segunda angulo del pendulo
sys =  [x(1);x(2);x(3);x(4)];
end
% end mdlOutputs

%
%=============================================================================
% mdlGetTimeOfNextVarHit
% Return the time of the next hit for this block.  Note that the result is
% absolute time.  Note that this function is only used when you specify a
% variable discrete-time sample time [-2 0] in the sample time array in
% mdlInitializeSizes.
%=============================================================================
%
function sys=mdlGetTimeOfNextVarHit(t,x,u)

sampleTime = 1;    %  Example, set the next hit to be one second later.
sys = t + sampleTime;
end
% end mdlGetTimeOfNextVarHit

%
%=============================================================================
% mdlTerminate
% Perform any end of simulation tasks.
%=============================================================================
%
function sys=mdlTerminate(t,x,u)

sys = [];
end

end




