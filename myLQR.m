function LQRsol = myLQR(t,x0,xd,sys,Q,R,P1)
% Solution to the Two-Point-Boundary-Problem
arguments
    t   % Timescale
    x0  % Initial state
    xd  % Desired state
    sys % LTI system (ss class)
    Q   % Q-matrix weight
    R   % R-matrix weight
    P1  % Final P-matrix weight
end
% Validate arguments
mustBeCompatibleWeight(Q,sys.A);
mustBeCompatibleWeight(R,sys.B);
mustBeCompatibleWeight(P1,sys.A);
mustBeCompatibleWeight(Q,x0);

% Initialize
N   = size(t,2);
x   = zeros(N,size(x0,2));
u   = zeros(N,1);
x(1,:) = x0;
dt  = t(2)-t(1);
t_f = t(end);
t_back= t(end:-1:1);

%Solve P, disregard returned time vector
[PsolT,PsolP] = ode45(@Pdot,t_back,reshape(P1,[],1));

% Reverse the backward order and reconstruct matrices
Plist = cell(1,N); 
for i=1:N 
    Plist{i}=reshape(PsolP(N-i+1,:),5,5);
end

% Solve x
for i = 2:N
    x(i,:) = ((x(i-1,:)')...
        + (sys.A-sys.B/R*(sys.B')*Plist{i})*(-xd'+x(i-1,:)')*dt)';
end

% Solve u
for i = 1:N
    u(i)   = -1/R*(sys.B')*Plist{i}*(-xd'+x(i,:)');
end

LQRsol = [t',x,u];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Local Functions
function Pout = Pdot(T,Y)
    P = reshape(Y,5,5);
    Pout = reshape(...
        -1*(P*sys.A+sys.A'*P-P*sys.B/R*sys.B'*P+Q), ...
        [],...
        1);
end
function mustBeSquareMatrix(in)
    if(size(in,1)~=size(in,2))
        eidType = 'mustBeSquareMatrix:notSquareMatrix';
        msgType = 'Input must be a NxN matrix';
        error(eidType,msgType)
    end
end
function mustBeCompatibleWeight(in1,in2)
    mustBeSquareMatrix(in1);
    if(size(in1,1)~=size(in2,1) && size(in1,2)~=size(in2,2))
        eidType = 'mustBeCompatibleWeight:notCompatible';
        msgType = 'Input1 must have a matching dimension with Input2';
        error(eidType,msgType)
    end
end
end