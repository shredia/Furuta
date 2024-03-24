function [sys,x0,str,ts,simStateCompliance] = PenduloFuruta(t,x,u,flag,xi,la,lp,ma,mp,M,J,g)


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
function sys=mdlDerivatives(t,x,u,la,lp,ma,mp,M,J,g)


x1=x(1);
x2=x(2);
x3=x(3);
x4=x(4);


a = J+((ma/3)+mp+M)*(la^2);
b = ((mp/3)+M)*(lp^2);
c = ((mp/2)+M)*la*lp; %y
d =  ((mp/2)+M)*g*lp;

sys(1) = x2;
sys(2)=(b*u-b*c*cos(x3)^2*sin(x3)*(x2)^2-2*b^2*cos(x3)*sin(x3)*x4+b*c*sin(x3)*(x4)^2-c*d*cos(x3)*sin(x3))/(a*b-c^2+(b^2+c^2)*sin(x3)^2);
sys(3)=x4;
sys(4)=(b*(a+b*sin(x3)^2)*cos(x3)*sin(x3)*(x2)^2+2*b*c*cos(x3)^2*sin(x3)*x4*x2-c^2*cos(x3)*sin(x3)*((x4)^2)-c*u*cos(x3))/(a*b-c^2+(b^2+c^2)*sin(x3)^2);
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




