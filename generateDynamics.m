function flySS = generateDynamics(dt)
% Generates the dynamics of a tethered yaw-free fruit fly (Elzinga et. al, 2012)
% Inputs:   Halteres and visual time delays (s). Defaults to Elzinga's
%           constants when omitted.
% Outputs:  State-space model that transforms a feed-forward asymmetry
%           into yaw velocity, and the equivalent transfer function

arguments
    dt (1,1) {mustBeNumeric};
end

% Elzinga's experimental constants
C_omega = 747;      % Damping coefficient
C_u     = 3530;     % Actuation coefficient      
I       = 1970;     % Moment of inertia
k_v     = 12.2;     % Visual gain
k_h     = 0.6;      % Halteres gain
omega_c = 0.7;      % Visual low pass filter cutoff frequency (Hz)
delta_h = 0.6*0.005;% Halteres time delay (s)
delta_v = 8*0.005;  % Visual time delay (s)

% Transfer functions of feedback loops
s           = tf('s');              % Transfer function variable
[VDN,VDD]   = pade(delta_v,1);      
visualD     = tf(VDN,VDD);          % Visual delay as 1st Pade approx.
[HDN,HDD]   = pade(delta_h,1);      
halteresD   = tf(HDN,HDD);          % Halteres delay as 1st Pade approx.
halteresOP  = k_h*C_u/(I*s+C_omega);% Halteres open loop
visualLPF   = tf( k_v*omega_c, ...
                  [1,omega_c]);     % Visual low pass filtering

% Feedback loop construction: Vision-guided flight is a nested feedback
% loop. The visual proportional controller reacts strongly, but only to 
% low frequencies (low pass filter), while the halteres P-control is
% weaker but responds to all frequencies.
halteresTF  = feedback(halteresOP, ...
                        halteresD); % Halteres feedback loop
visualTF    = feedback(halteresTF, ...
                        visualD * visualLPF ...
                        );              % Visual feedback loop

% Build State-Space model
[fA,fB,fC,fD]   = tf2ss( ...
                    visualTF.Numerator{1}, ...
                    visualTF.Denominator{1} ...
                    );
flySS           = ss(fA,fB,fC,fD);

% With the default delays:
% dx0dt = 
% dx1dt = -3.587*x1 +1.653*x2   +23.24*x3   -8.5000*uff
% dx2dt =        x1
% dx3dt =        x2
% dx4dt =        x3
% y     = +1.075*x1 +4.605*x2   +3.593*x3   +0.6272*x4
% IN OTHER WORDS:
% x'''' = -3.587*x'''   +1.653*x''  +23.24*x'   -8.5000*uff
% y     = +1.075*x'''   +4.605*x''  +3.593*x'   -0.6272*x    

flySS = chgTimeUnit(flySS,'milliseconds');  % Use appropriate timescale

% Now let's append yaw as a state

% The "state" of this system isn't analogous to any physical property
% However, for convenience I will rebuild the A and B matrices to 
% include Yaw Angle as a state. I can do this because the system also
% gives me a definition for the time derivative of the position:
% velocity, which is = C*x. If position is first,
A = [flySS.C; flySS.A];            % C can be stacked on top of A...
A =[zeros(size(flySS.A,1)+1,1), A];% ...And add a zero vector to show 
                                   % that position does not influence
                                   % any state.
B = [0;flySS.B];                   % Control does not (directly)
                                   % change velocity
C = [0,flySS.C];
flySS = ss(A,B,C,0,dt); flySS.TimeUnit="milliseconds";
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% References
% [1] Elzinga Michael J., Dickson William B. and Dickinson Michael H.
%       2012 The influence of sensory delay on the yaw dynamics of a 
%       flapping insect J. R. Soc. Interface.91685–1696
%       http://doi.org/10.1098/rsif.2011.0699
end