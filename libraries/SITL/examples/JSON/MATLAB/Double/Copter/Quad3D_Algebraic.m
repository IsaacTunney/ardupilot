function [SolutionToAlgebraicEquations,Output] = Quad3D_Algebraic(t, Params, State, Command)
%===========================================================================
% File: Quad3D_Algebraic.m created Nov 11 2023 by MotionGenesis 6.1.
% Portions copyright (c) 2009-2021 Motion Genesis LLC.  Rights reserved.
% MotionGenesis Basic Research/Vanilla Licensee: John Bass. (until October 2025).
% Paid-up MotionGenesis Basic Research/Vanilla licensees are granted the right
% to distribute this code for legal academic (non-professional) purposes only,
% provided this copyright notice appears in all copies and distributions.
%===========================================================================
% The software is provided "as is", without warranty of any kind, express or    
% implied, including but not limited to the warranties of merchantability or    
% fitness for a particular purpose. In no event shall the authors, contributors,
% or copyright holders be liable for any claim, damages or other liability,     
% whether in an action of contract, tort, or otherwise, arising from, out of, or
% in connection with the software or the use or other dealings in the software. 
%===========================================================================
I1Dt=0; I2Dt=0; I3Dt=0; I4Dt=0; pDt=0; qDt=0; rDt=0; w1Dt=0; w2Dt=0; w3Dt=0; w4Dt=0; xDDt=0; yDDt=0; zDDt=0; Area=0; Thr1=0; Thr2=0;
Thr3=0; Thr4=0; Tor1=0; Tor2=0; Tor3=0; Tor4=0; airspeedMag=0; batt_current=0; batt_dropped_voltage=0; Command_V1=0; Command_V2=0;
Command_V3=0; Command_V4=0; Command_w1=0; Command_w2=0; Command_w3=0; Command_w4=0; drag=0; planarAirspeed=0; thrCorrFactor=0; tilt=0;
TorAccel1=0; TorAccel2=0; TorAccel3=0; TorAccel4=0; TorMec1=0; TorMec2=0; TorMec3=0; TorMec4=0; V_BEMF1=0; V_BEMF2=0; V_BEMF3=0; V_BEMF4=0;
V_diff_1=0; V_diff_2=0; V_diff_3=0; V_diff_4=0; xAirspeed=0; yAirspeed=0;


%-------------------------------+--------------------------+-------------------+-----------------
% Quantity                      | Value                    | Units             | Description
%-------------------------------|--------------------------|-------------------|-----------------
g                               =  Params.g;               % m/s^2               Constant
mB                              =  Params.mB;              % kg                  Constant
IBxx                            =  Params.IBxx;            % kg*m^2              Constant
IByy                            =  Params.IByy;            % kg*m^2              Constant
IBzz                            =  Params.IBzz;            % kg*m^2              Constant
dxm                             =  Params.dxm;             % m                   Constant
dym                             =  Params.dym;             % m                   Constant

c0                              =  Params.c0;              % UNITS               Constant
c1                              =  Params.c1;              % UNITS               Constant
kTh                             =  Params.kTh;             % NoUnits             Constant
kTo                             =  Params.kTo;             % NoUnits             Constant
kv                              =  Params.kv;              % N*m/Amp             Constant
kt                              =  Params.kt;              % Volt/(rad/s)        Constant
Jm                              =  Params.Jm;              % kg*m^2              Constant
b                               =  Params.b;               % N*m/(rad/s)         Constant
L                               =  Params.L;               % Henry               Constant
Res                             =  Params.Res;             % Ohms                Constant

thrVelCoeff                     =  Params.thrVelCoeff;     % UNITS               Constant
efpa0_tilt                      =  Params.efpa0_tilt;      % UNITS               Constant
efpa1_tilt                      =  Params.efpa1_tilt;      % UNITS               Constant
efpa2_tilt                      =  Params.efpa2_tilt;      % UNITS               Constant
efpa0_vel                       =  Params.efpa0_vel;       % UNITS               Constant
efpa1_vel                       =  Params.efpa1_vel;       % UNITS               Constant
Area                            =  Params.Area;            % m^2                 Constant
Cd                              =  Params.Cd;              % NoUnits             Constant
rho                             =  Params.rho;             % kg/m^3              Constant

batt_resistance                 =  Params.batt_resistance; % Ohms                Constant
batt_voltage                    =  Params.batt_voltage;    % Volt                Constant



e0                              =  State(1);               % UNITS               Initial Value
e1                              =  State(2);               % UNITS               Initial Value
e2                              =  State(3);               % UNITS               Initial Value
e3                              =  State(4);               % UNITS               Initial Value
I1                              =  State(5);               % Amp                 Initial Value
I2                              =  State(6);               % Amp                 Initial Value
I3                              =  State(7);               % Amp                 Initial Value
I4                              =  State(8);               % Amp                 Initial Value
p                               =  State(9);               % rad/s               Initial Value
q                               =  State(10);              % rad/s               Initial Value
r                               =  State(11);              % rad/s               Initial Value
w1                              =  State(12);              % rad/s               Initial Value
w2                              =  State(13);              % rad/s               Initial Value
w3                              =  State(14);              % rad/s               Initial Value
w4                              =  State(15);              % rad/s               Initial Value
x                               =  State(16);              % m                   Initial Value
y                               =  State(17);              % m                   Initial Value
z                               =  State(18);              % m                   Initial Value
xDt                             =  State(19);              % m/s                 Initial Value
yDt                             =  State(20);              % m/s                 Initial Value
zDt                             =  State(21);              % m/s                 Initial Value

qW                              =  0.0;                    % UNITS               Constant
WindVel                         =  0.0;                    % UNITS               Constant

Command_percent1                =  Command(1);             % UNITS               Constant
Command_percent2                =  Command(2);             % UNITS               Constant
Command_percent3                =  Command(3);             % UNITS               Constant
Command_percent4                =  Command(4);             % UNITS               Constant
%-------------------------------+--------------------------+-------------------+-----------------


SolutionToAlgebraicEquations = DoCalculations;
Output = CalculateOutput;


%===========================================================================
function SolutionToAlgebraicEquations = DoCalculations
%===========================================================================
planarAirspeed = sqrt((WindVel*sin(qW)-yDt)^2+(WindVel*cos(qW)-xDt)^2);
thrCorrFactor = 1 - thrVelCoeff*planarAirspeed;
thrCorrFactor = min([1, max([0, thrCorrFactor])]);

Thr1 = kTh*w1^2*thrCorrFactor;
Thr2 = kTh*w2^2*thrCorrFactor;
Thr3 = kTh*w3^2*thrCorrFactor;
Thr4 = kTh*w4^2*thrCorrFactor;
Tor1 = kTo*w1^2;
Tor2 = kTo*w2^2;
Tor3 = kTo*w3^2;
Tor4 = kTo*w4^2;

tilt = acos(-1+2*e0^2+2*e3^2);
% Area = efpa0_vel + efpa1_vel*planarAirspeed;
Area = efpa0_tilt + efpa1_tilt*tilt + efpa2_tilt*tilt^2;
drag = 0.5*Cd*rho*Area*(WindVel^2+xDt^2+yDt^2+zDt^2-2*WindVel*sin(qW)*yDt-2*WindVel*cos(qW)*xDt);

batt_current = I1 + I2 + I3 + I4;
batt_dropped_voltage = batt_voltage - batt_resistance*batt_current;

Command_w1 = c0 + c1*Command_percent1;
Command_w2 = c0 + c1*Command_percent2;
Command_w3 = c0 + c1*Command_percent3;
Command_w4 = c0 + c1*Command_percent4;

Command_w1 = min([Params.wMaxNoload, max([0, Command_w1])]);
Command_w2 = min([Params.wMaxNoload, max([0, Command_w2])]);
Command_w3 = min([Params.wMaxNoload, max([0, Command_w3])]);
Command_w4 = min([Params.wMaxNoload, max([0, Command_w4])]);

Command_V1 = kv*Command_w1*batt_dropped_voltage/batt_voltage;
V_BEMF1 = kv*w1;
V_diff_1 = Command_V1 - V_BEMF1;
TorMec1 = kt*I1;
TorAccel1 = TorMec1 - Tor1;

Command_V2 = kv*Command_w2*batt_dropped_voltage/batt_voltage;
V_BEMF2 = kv*w2;
V_diff_2 = Command_V2 - V_BEMF2;
TorMec2 = kt*I2;
TorAccel2 = TorMec2 - Tor2;

Command_V3 = kv*Command_w3*batt_dropped_voltage/batt_voltage;
V_BEMF3 = kv*w3;
V_diff_3 = Command_V3 - V_BEMF3;
TorMec3 = kt*I3;
TorAccel3 = TorMec3 - Tor3;

Command_V4 = kv*Command_w4*batt_dropped_voltage/batt_voltage;
V_BEMF4 = kv*w4;
V_diff_4 = Command_V4 - V_BEMF4;
TorMec4 = kt*I4;
TorAccel4 = TorMec4 - Tor4;

COEF = zeros( 14, 14 );
COEF(1,1) = mB;
COEF(2,2) = mB;
COEF(3,3) = mB;
COEF(4,4) = IBxx*(-1+2*e0^2+2*e1^2);
COEF(4,5) = -2*IByy*(e0*e3-e1*e2);
COEF(4,6) = 2*IBzz*(e0*e2+e1*e3);
COEF(5,4) = 2*IBxx*(e0*e3+e1*e2);
COEF(5,5) = IByy*(-1+2*e0^2+2*e2^2);
COEF(5,6) = -2*IBzz*(e0*e1-e2*e3);
COEF(6,4) = -2*IBxx*(e0*e2-e1*e3);
COEF(6,5) = 2*IByy*(e0*e1+e2*e3);
COEF(6,6) = IBzz*(-1+2*e0^2+2*e3^2);
COEF(7,11) = L;
COEF(8,7) = Jm;
COEF(9,12) = L;
COEF(10,8) = Jm;
COEF(11,13) = L;
COEF(12,9) = Jm;
COEF(13,14) = L;
COEF(14,10) = Jm;
RHS = zeros( 1, 14 );
RHS(1) = 1000*(WindVel*cos(qW)-xDt)*drag/(1+1000*sqrt(WindVel^2+xDt^2+yDt^2+zDt^2-2*WindVel*sin(qW)*yDt-2*WindVel*cos(qW)*xDt))  ...
- 2*(Thr1+Thr2+Thr3+Thr4)*(e0*e2+e1*e3);
RHS(2) = 2*(Thr1+Thr2+Thr3+Thr4)*(e0*e1-e2*e3) + 1000*(WindVel*sin(qW)-yDt)*drag/(1+1000*sqrt(WindVel^2+xDt^2+yDt^2+zDt^2-2*  ...
WindVel*sin(qW)*yDt-2*WindVel*cos(qW)*xDt));
RHS(3) = g*mB - (Thr1+Thr2+Thr3+Thr4)*(-1+2*e0^2+2*e3^2) - 1000*zDt*drag/(1+1000*sqrt(WindVel^2+xDt^2+yDt^2+zDt^2-2*WindVel*sin(qW)*  ...
yDt-2*WindVel*cos(qW)*xDt));
RHS(4) = 2*(e0*e2+e1*e3)*((IBxx-IByy)*p*q+TorMec1+TorMec2-TorMec3-TorMec4) - 2*(e0*e3-e1*e2)*(dxm*Thr1+dxm*Thr3-dxm*Thr2-dxm*Thr4-(  ...
IBxx-IBzz)*p*r) - (-1+2*e0^2+2*e1^2)*(dym*Thr1+dym*Thr4-dym*Thr2-dym*Thr3-(IByy-IBzz)*q*r);
RHS(5) = (-1+2*e0^2+2*e2^2)*(dxm*Thr1+dxm*Thr3-dxm*Thr2-dxm*Thr4-(IBxx-IBzz)*p*r) - 2*(e0*e3+e1*e2)*(dym*Thr1+dym*Thr4-dym*Thr2-dym*  ...
Thr3-(IByy-IBzz)*q*r) - 2*(e0*e1-e2*e3)*((IBxx-IByy)*p*q+TorMec1+TorMec2-TorMec3-TorMec4);
RHS(6) = 2*(e0*e1+e2*e3)*(dxm*Thr1+dxm*Thr3-dxm*Thr2-dxm*Thr4-(IBxx-IBzz)*p*r) + 2*(e0*e2-e1*e3)*(dym*Thr1+dym*Thr4-dym*Thr2-dym*  ...
Thr3-(IByy-IBzz)*q*r) + (-1+2*e0^2+2*e3^2)*((IBxx-IByy)*p*q+TorMec1+TorMec2-TorMec3-TorMec4);
RHS(7) = V_diff_1 - Res*I1;
RHS(8) = TorAccel1 - b*w1;
RHS(9) = V_diff_2 - Res*I2;
RHS(10) = TorAccel2 - b*w2;
RHS(11) = V_diff_3 - Res*I3;
RHS(12) = TorAccel3 - b*w3;
RHS(13) = V_diff_4 - Res*I4;
RHS(14) = TorAccel4 - b*w4;
SolutionToAlgebraicEquations = COEF \ transpose(RHS);

% Update variables after uncoupling equations
xDDt = SolutionToAlgebraicEquations(1);
yDDt = SolutionToAlgebraicEquations(2);
zDDt = SolutionToAlgebraicEquations(3);
pDt = SolutionToAlgebraicEquations(4);
qDt = SolutionToAlgebraicEquations(5);
rDt = SolutionToAlgebraicEquations(6);
w1Dt = SolutionToAlgebraicEquations(7);
w2Dt = SolutionToAlgebraicEquations(8);
w3Dt = SolutionToAlgebraicEquations(9);
w4Dt = SolutionToAlgebraicEquations(10);
I1Dt = SolutionToAlgebraicEquations(11);
I2Dt = SolutionToAlgebraicEquations(12);
I3Dt = SolutionToAlgebraicEquations(13);
I4Dt = SolutionToAlgebraicEquations(14);

airspeedMag = sqrt(WindVel^2+xDt^2+yDt^2+zDt^2-2*WindVel*sin(qW)*yDt-2*WindVel*cos(qW)*xDt);
xAirspeed = xDt - WindVel*cos(qW);
yAirspeed = yDt - WindVel*sin(qW);


end


%===========================================================================
function Output = CalculateOutput
%===========================================================================
Output = zeros( 1, 50 );
Output(1) = t;
Output(2) = x;
Output(3) = y;
Output(4) = z;
Output(5) = xDt;
Output(6) = yDt;
Output(7) = zDt;
Output(8) = xDDt;
Output(9) = yDDt;
Output(10) = zDDt;
Output(11) = e0;
Output(12) = e1;
Output(13) = e2;
Output(14) = e3;
Output(15) = p;
Output(16) = q;
Output(17) = r;
Output(18) = pDt;
Output(19) = qDt;
Output(20) = rDt;

Output(21) = Thr1;
Output(22) = Thr2;
Output(23) = Thr3;
Output(24) = Thr4;
Output(25) = Tor1;
Output(26) = Tor2;
Output(27) = Tor3;
Output(28) = Tor4;
Output(29) = TorMec1;
Output(30) = TorMec2;
Output(31) = TorMec3;
Output(32) = TorMec4;

Output(33) = w1;
Output(34) = w2;
Output(35) = w3;
Output(36) = w4;
Output(37) = w1Dt;
Output(38) = w2Dt;
Output(39) = w3Dt;
Output(40) = w4Dt;
Output(41) = I1;
Output(42) = I2;
Output(43) = I3;
Output(44) = I4;

Output(45) = qW;
Output(46) = WindVel;

Output(47) = airspeedMag;
Output(48) = xAirspeed;
Output(49) = yAirspeed;
Output(50) = zDt;
end

%=========================================
end    % End of function Quad3D_Algebraic
%=========================================
