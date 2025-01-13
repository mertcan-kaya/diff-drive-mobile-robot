clc, clear all, close all


% Define parameters for Dijkstra and Dynamic Window Approach
parameters.dist_threshold = 0.25; % threshold distance to goal
parameters.angle_threshold = 0.1; % threshold orientation to goal

parameters.wheelDiameter = 0.05; % meter
parameters.wheelRadius = parameters.wheelDiameter/2.0;
parameters.interWheelDistance = 0.14; % meter

% controller parameters
parameters.Krho = 0.5;
parameters.Kalpha = 1.5;
parameters.Kbeta = -0.6;
parameters.backwardAllowed = false;
parameters.useConstantSpeed = true;
parameters.constantSpeed = 0.4;

timePrv = 0;

wheelRPosPrv = 0.0;
wheelLPosPrv = 0.0;

x = 0.0;
y = 0.0;
theta = 0.0;

xg = 1.0;
yg = 0.0;
thetag = 0.0;

wheelLPos = 0.351;
wheelRPos = 0.368;

timeStp = 0.004;

wheelLVel = (wheelLPos-wheelLPosPrv)/timeStp;
wheelRVel = (wheelRPos-wheelRPosPrv)/timeStp;

xR_dot = parameters.wheelRadius*(wheelRVel+wheelLVel)/2;
theta_dot = parameters.wheelRadius*(wheelRVel-wheelLVel)/(2*parameters.interWheelDistance);

theta = theta + theta_dot*timeStp;

x_dot = cos(theta)*xR_dot;
y_dot = sin(theta)*xR_dot;

x = x + x_dot*timeStp;
y = y + y_dot*timeStp;

% run control step
[ vu, omega ] = calculateControlOutput([x, y, theta], [xg, yg, thetag], parameters)

% Calculate wheel speeds
[LeftWheelVelocity, RightWheelVelocity ] = calculateWheelSpeeds(vu, omega, parameters)

rho = sqrt((xg-x)^2+(yg-y)^2);          % pythagoras theorem, sqrt(dx^2 + dy^2)
lambda = atan2(yg-y, xg-x);             % angle of the vector pointing from the robot to the goal in the inertial frame
alpha = normalizeAngle(lambda - theta); % angle of the vector pointing from the robot to the goal in the robot frame

beta = thetag-lambda;

if parameters.backwardAllowed % Backward speed allowed
    % If obstacle in "front" => go forward
    if(abs(alpha) > pi/2)
        alpha = normalizeAngle(lambda-theta-pi);
        beta = thetag-lambda-pi;
        parameters.Krho = -parameters.Krho;
    end
end

vu = parameters.Krho*rho % [m/s]
omega = parameters.Kalpha*alpha + parameters.Kbeta*beta % [rad/s]

if parameters.useConstantSpeed % Constant speed enabled
    absVel = abs(vu);
    if(absVel>1e-6)
        vu = parameters.constantSpeed*vu/absVel;
        omega = parameters.constantSpeed*omega/absVel;
    end
end

Phia = vu/parameters.wheelRadius;
Phib = omega*parameters.interWheelDistance/(parameters.wheelDiameter);

LeftWheelVelocity = Phia - Phib
RightWheelVelocity = Phia + Phib

% End condition
dtheta = abs(normalizeAngle(theta-thetag));

EndCond = (rho < parameters.dist_threshold && dtheta < parameters.angle_threshold) || rho > 5;    
  
function [angle1] = normalizeAngle(angle)
    %normalizeAngle   set angle to the range [-pi,pi)

    angle1 = mod( angle+pi, 2*pi) - pi;
end

function [ vu, omega ] = calculateControlOutput( robotPose, goalPose, parameters )
%CALCULATECONTROLOUTPUT This function computes the motor velocities for a differential driven robot

% current robot position and orientation
x = robotPose(1);
y = robotPose(2);
theta = robotPose(3);

% goal position and orientation
xg = goalPose(1);
yg = goalPose(2);
thetag = goalPose(3);

% compute control quantities
rho = sqrt((xg-x)^2+(yg-y)^2);  % pythagoras theorem, sqrt(dx^2 + dy^2)
lambda = atan2(yg-y, xg-x);     % angle of the vector pointing from the robot to the goal in the inertial frame
alpha = lambda - theta;         % angle of the vector pointing from the robot to the goal in the robot frame
alpha = normalizeAngle(alpha);

if parameters.backwardAllowed % Backward speed allowed
    % If obstacle in "front" => go forward
    if(abs(alpha)<=pi/2)
        beta = thetag-lambda;
        Krho2 = parameters.Krho;
    else
        alpha = lambda-theta-pi;
        alpha = normalizeAngle(alpha);
        beta = thetag-lambda-pi;
        Krho2 = -parameters.Krho;
    end
else
    beta = thetag-lambda;
    Krho2 = parameters.Krho;
end
beta = normalizeAngle(beta);

vu = Krho2 * rho; % [m/s]
omega = parameters.Kalpha * alpha + parameters.Kbeta * beta; % [rad/s]

if parameters.useConstantSpeed % Constant speed enabled
    absVel = abs(vu);
    if(absVel>1e-6)
        vu = vu/absVel*parameters.constantSpeed;
        omega = omega/absVel*parameters.constantSpeed;
    end
end


end
function [ LeftWheelVelocity, RightWheelVelocity ] = calculateWheelSpeeds( vu, omega, parameters )
%CALCULATEWHEELSPEEDS This function computes the motor velocities for a differential driven robot

wheelRadius = parameters.wheelRadius;
halfWheelbase = parameters.interWheelDistance/2;

M = [wheelRadius/2                wheelRadius/2;
     wheelRadius/2/halfWheelbase -wheelRadius/2/halfWheelbase];
Minv = inv(M);
PhiPrime = Minv * [vu; omega];
LeftWheelVelocity = PhiPrime(2);
RightWheelVelocity = PhiPrime(1);
end
