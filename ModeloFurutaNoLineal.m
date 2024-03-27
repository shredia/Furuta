function [sys,x0,str,ts,simStateCompliance] = ModeloFurutaNoLineal(t,x,u,flag,xi,g,Phi,dPhi,Tau,dTau,J,M_2,l_bi,C_x,C_z,I_x,I_z,B_p,B_u,d)


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
sizes.NumOutputs     = 2; %salidas
sizes.NumInputs      = 2; %entradas
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
%Phi==>thetas//////listo
%
function sys=mdlDerivatives(t,x,u,g,Phi,dPhi,Tau,Tau_fu,Tau_fp,J,M_2,l_bi,C_x,C_z,I_x,I_z,B_p,B_u,d,theta,dtheta)
%se definen las matrices M(theta)[2x2], D(theta,Phi)[2x2] F(theta)[2x1]
%U[2x1]
%%%%%SE ASUME QUE t=theta%%%%%%%%%

%ecuaciones matriz M(theta)
Mtheta1_1= J+M_2*l_bi^2+2*M_2*l_bi*C_x+I_z+I_x*sin(theta)*sin(theta);
Mtheta2_1= M_2*l_bi*C_z*cos(theta);
Mtheta1_2= M_2*l_bi*C_z*cos(theta);
Mtheta2_2= I_x;

% matriz M(theta)
M(theta)=[Mtheta1_1,Mtheta2_1;Mtheta1_2,Mtheta2_2];

%ecuaciones D(theta,Phi)

DthetaPhi1_1=2*I_x*dtheta*sin(theta)*cos(theta);
DthetaPhi1_2=-dPhi*I_x*cos(theta)*sin(theta);
DthetaPhi2_1= M_2*l_bi*C_z*sin(theta)*dtheta;
DthetaPhi2_2=0;

% matriz D(theta,Phi)
D(theta_Phi)=[DthetaPhi1_1,DthetaPhi2_1;DthetaPhi1_2,DthetaPhi2_2];

%ecuaciones F(theta)

Ftheta1_1=0;
Ftheta1_2=M_2*g*C_z*sin(theta);

%matriz F(theta)

F(theta)=[Ftheta1_1;Ftheta1_2];
    
%ecuaciones U

U_1=Tau+T_fu;
U_2=Tau_fp;

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




