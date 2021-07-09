clear all; close all; clc;

disp('ME577_Project_Group8_driver.m - Project')
disp('ME577 - ADVANCED LINEAR CONTROLS (SPRING 2020)')
disp('April 28, 2020')
disp('================================================')
disp('JORDAN BAGWELL')
disp('JEFFREY MAYS')
disp('NICHOLAS SCIORTINO')
disp('THE UNIVERSITY OF ALABAMA')


%% User Defined Parameters

% Scipt Parameters
doWeightOptimization = true;
runSimulations = true;
plotResults = false;
showAnimation = true;

% Simulation Parameters
dt = .05;
tEnd = 5;

% Disturbance Parameters
velocity_limit = 3;
angle_limit = 10;
angle_dot_limit = 3;
chanceDisturbance = 0.25;

%                       List of Scenarios
%-------------------------------------------------------------
% case 1  % no disturbance
% case 2  % step of 1 m cart position     
% case 3  % ramp of 1 m/s 
% case 4  % impulse of 1 m/s cart velocity      
% case 5  % impulse of 1 rad/s arm angular velocity  
% case 6  % repeated impulse of 1 m/s velocity       
% case 7  % repeated impulse of 1 rad/s arm angular velocity        
% case 8  % ramp of 1 m/s^2 cart velocity
% case 9  % ramp of 1 rad/s^2 arm angular velocity       
% case 10 % random disturbance


%% Prepare Workspace
global K
global R
global sys
global m1
global m2
global L
global RTOD
global DTOR
global disturbanceCase
global thetaMax
global x0
global SIMULATION_NAME
%#ok<*NOPTS>
%#ok<*UNRCH>

RTOD = 180.0 / pi;      % radians to degrees conversion
DTOR = pi / 180.0;      % degrees to radians conversion
thetaMax = 100 * DTOR;  % angle at which pendulum hits ground
SIMULATION_NAME = 'inverted_pendulum_linear_model';
numCases = 10;
scenarioParams = [velocity_limit,...    
                  angle_dot_limit,... 
                  chanceDisturbance];

% Define Problem Values
m1  = 2.20;     % kg, mass of cart
m2  = 0.35;     % kg, mass of bar
L   = 1.3;      % m, length of bar
b   = 0.25;     % N/m-s, cart friction coeff.
g   = 9.795;    % m/s^2, gravity accel in Tuscaloosa (yes, really)

% Retreive LTI state space matrices
[A, B, C, D] = inv_pend_SS(m1, m2, L, b, g);

% Linear Analysis
eig(A)
Controlability = rank(ctrb(A,B)) 
Observability = rank([C; C*A; C*A*A])

% Create Scenarios Datasets
for disturbanceCase = 1:10
    [disturbanceCases{disturbanceCase}, systemSetpointsCases{disturbanceCase}] = ...
        createScenario(disturbanceCase, dt,tEnd,scenarioParams); 
end


%% Design System Controllers
% Pole Placement
des_eigs = [-2.219+1i*1.4667,...
            -2.219-1i*1.4667,...
            -1.5018+1i*4.4761,...
            -1.5018-1i*4.4761];
K_PP = place(A, B, des_eigs);

% Design LQR controller
posCost = 100;          % cost of linear position error
velCost = 10;           % cost of linear velocity error
angCost = 1;            % cost of arm angle error
angVelCost = 100;       % cost of arm angular velocity error

R = 1;                  % cost of input energy

Q = [posCost 0 0 0;     % state cost matrix
    0 velCost 0 0;
    0 0 angCost 0;
    0 0 0 angVelCost];

sys = ss(A,B,C,D);

K_LQR = lqr(sys,Q,R);

if doWeightOptimization
    % get initial weight simulation results
    disturbanceData = disturbanceCases{2};  disturbanceCase = 2;
    systemSetpoints = systemSetpointsCases{2};
    K_LQR_init = K_LQR
    K = K_LQR;
    QR_initial_results = sim(SIMULATION_NAME);
    Qinit = Q;
    % Run QR Optimization Algorithm
    weights = [posCost, velCost, angCost, angVelCost];
    [QNew, R] = QR_Optimization(weights,A,B);
    K_LQR = lqr(A,B,QNew,R);
    K_LQR_new = K_LQR
    Qinit
    QNew
    % Run New Simulation
    QR_optimization_results = sim(SIMULATION_NAME);
    
    if plotResults
        doubleAnalysis('QR Controller Comparison',...
                       'Initial QR Controller', 'Optimizaed QR Controller',...
                       QR_initial_results,QR_optimization_results)
    end
    if showAnimation
       doubleAnimation('QR Optimization Animation',...
                       'Initial QR Controller', 'Optimized QR Controller',...
                       QR_initial_results, QR_optimization_results)
    end
end


%% Run Simulations
if runSimulations
    numSims = numCases * 2 + 1;
    simNum = 0;
    h = waitbar(simNum/numSims,'Running Simulations');

    for caseNum = 1:numCases
        disturbanceCase = caseNum;
        disturbanceData = disturbanceCases{caseNum};
        systemSetpoints = systemSetpointsCases{caseNum};

        K = K_LQR;
        LQR_simulation_results{caseNum} = sim(SIMULATION_NAME);
        simNum = simNum + 1;    waitbar(simNum/numSims, h)
        K = K_PP;
        PP_simulation_results{caseNum} = sim(SIMULATION_NAME);
        simNum = simNum + 1;    waitbar(simNum/numSims, h)
    end
    % Free Response Simulation
    K = K_LQR .* 0;
    disturbanceCase = 1;    disturbanceData = disturbanceCases{1};  
      systemSetpoints = systemSetpointsCases{1};
      x0 = [0 0 .01 0];
      free_response_results = sim(SIMULATION_NAME);
      simNum = simNum + 1;    waitbar(simNum/numSims, h)

    close(h)
end


%% Analysis
% Plot System Responses
if plotResults && runSimulations
    disturbanceCase = 1;
    singleAnalysis('Initial Pendulum Arm Angle Free Response',...
                   'No Control',...
                   free_response_results)
    disturbanceCase = 2;
    doubleAnalysis('Position Step Response',...
                   'LQR Controller', 'Pole Placement Controller',...
                   LQR_simulation_results{2}, PP_simulation_results{2})
    disturbanceCase = 3;
    doubleAnalysis('Cart Position Ramp Response', ...
                   'LQR Controller', 'Pole Placement Controller',...
                   LQR_simulation_results{3}, PP_simulation_results{3})
    disturbanceCase = 4
    doubleAnalysis('Cart Velocity Impulse Disturbance', ...
                   'LQR Controller', 'Pole Placement Controller',...
                   LQR_simulation_results{4}, PP_simulation_results{4})
    disturbanceCase = 5;
    doubleAnalysis('Pendulum Angular Velocity Impulse Response',...
                   'LQR Controller', 'Pole Placement Controller',...
                   LQR_simulation_results{5}, PP_simulation_results{5})   
    disturbanceCase = 6;
    doubleAnalysis('Cart Velocity Repeated Impulse Response',...
                   'LQR Controller', 'Pole Placement Controller',...
                   LQR_simulation_results{6}, PP_simulation_results{6})    
    disturbanceCase = 7;
    doubleAnalysis('Pendulum Angular Velocity Repeated Impulse Response',...
                   'LQR Controller', 'Pole Placement Controller',...
                   LQR_simulation_results{7}, PP_simulation_results{7})
    disturbanceCase = 8;
    doubleAnalysis('Ramp of 1 m/s^2 Velocity Disturbance',...
                   'LQR Controller', 'Pole Placement Controller',...
                   LQR_simulation_results{8}, PP_simulation_results{8})
    disturbanceCase = 9;
    doubleAnalysis('Ramp of 1 rad/s^2 Angular Velocity Disturbance',...
                   'LQR Controller', 'Pole Placement Controller',...
                   LQR_simulation_results{9}, PP_simulation_results{9})
    disturbanceCase = 10;
    doubleAnalysis('Random Disturbance Response',...
                   'LQR Controller', 'Pole Placement Controller',...
                   LQR_simulation_results{10}, PP_simulation_results{10})               
end


%% Run Response Animations
if showAnimation && runSimulations
    
    disturbanceCase = 1;
    singleAnnimation('Free Response Animation','Free Response Animation',...
                      free_response_results)
    disturbanceCase = 2;
    doubleAnimation('Position Step Input Response',...
                     'LQR Controller', 'Pole Placement Controller',...
                     LQR_simulation_results{2}, PP_simulation_results{2})
    disturbanceCase = 3;
    doubleAnimation('Position Ramp Input Response',...
                     'LQR Controller', 'Pole Placement Controller',...
                     LQR_simulation_results{3}, PP_simulation_results{3})
    disturbanceCase = 4;
    doubleAnimation('Cart Velocity Impulse Disturbance',...
                     'LQR Controller', 'Pole Placement Controller',...
                     LQR_simulation_results{4}, PP_simulation_results{4})
    disturbanceCase = 5;
    doubleAnimation('Pendulum Angular Velocity Impulse Simulation',...
                     'LQR Controller', 'Pole Placement Controller',...
                     LQR_simulation_results{5}, PP_simulation_results{5})
    disturbanceCase = 6;
    doubleAnimation('Cart Velocity Repeated Impulse Disturbance',...
                     'LQR Controller', 'Pole Placement Controller',...
                     LQR_simulation_results{6}, PP_simulation_results{6})
    disturbanceCase = 7;
    doubleAnimation('Pendulum Angular Velocity Repeated Impulse Disturbance',...
                     'LQR Controller', 'Pole Placement Controller',...
                     LQR_simulation_results{7}, PP_simulation_results{7})
                 
    disturbanceCase = 8;
    doubleAnimation('Pendulum Velocity Ramp Disturbance',...
                     'LQR Controller', 'Pole Placement Controller',...
                     LQR_simulation_results{8}, PP_simulation_results{8})
                 
    disturbanceCase = 9;
    doubleAnimation('Pendulum Angular Velocity Ramp Disturbance',...
                     'LQR Controller', 'Pole Placement Controller',...
                     LQR_simulation_results{9}, PP_simulation_results{9})                 
    disturbanceCase = 10;
    doubleAnimation('Random Disturbance Animation',...
                     'LQR Controller', 'Pole Placement Controller',...
                     LQR_simulation_results{10}, PP_simulation_results{10})
end


%% Supporting Functions

function doubleAnalysis(figName,sys1Name,sys2Name,sys1Data,sys2Data)
global RTOD
global disturbanceCase

figure('Name', figName)
set(gcf, 'Position', [100 100 800 600])

subplot(5,1,1)
hold on
plot(sys1Data.results.position, 'k', 'LineWidth', 1.5)
plot(sys2Data.results.position, 'r', 'LineWidth', 1.5)
ylabel('Position (m)')
if (any(disturbanceCase == 2))
    sys1tSettle = findSettlingTime(sys1Data)*ones(1,100);
    sys2tSettle = findSettlingTime(sys2Data)*ones(1,100);
    maxPosition = max(max(abs(sys1Data.results.position.data),...
                       abs(sys2Data.results.position.data)));
    
    plot(sys1tSettle, linspace(-maxPosition, maxPosition,100),'--k')
    plot(sys2tSettle, linspace(-maxPosition, maxPosition,100),'--r')
    legend(sys1Name,sys2Name,'Settling Time','Settling Time');
else 
    legend(sys1Name,sys2Name)
end

subplot(5,1,2)
hold on
plot(sys1Data.results.velocity, 'k', 'LineWidth', 1.5)
plot(sys2Data.results.velocity, 'r', 'LineWidth', 1.5)
if any(disturbanceCase == 3)
sys1tSettle = findSettlingTime(sys1Data)*ones(1,100);
    sys2tSettle = findSettlingTime(sys2Data)*ones(1,100);
    maxVelocity = max(max(abs(sys1Data.results.velocity.data),...
                       abs(sys2Data.results.velocity.data)));
    
    plot(sys1tSettle, linspace(-maxVelocity, maxVelocity,100),'--k')
    plot(sys2tSettle, linspace(-maxVelocity, maxVelocity,100),'--r')
    legend(sys1Name,sys2Name,'Settling Time','Settling Time'); 
end
ylabel('Velocity (m/s)')

subplot(5,1,3)
hold on
plot(sys1Data.results.angle * RTOD, 'k', 'LineWidth', 1.5)
plot(sys2Data.results.angle * RTOD, 'r', 'LineWidth', 1.5)
ylabel('Angle (deg)')


subplot(5,1,4)
hold on
plot(sys1Data.results.angleRate * RTOD, 'k', 'LineWidth', 1.5)
plot(sys2Data.results.angleRate * RTOD, 'r', 'LineWidth', 1.5)
ylabel('Angular Rate (deg/s)')

subplot(5,1,5)
hold on
plot(sys1Data.results.force, 'k', 'LineWidth', 1.5)
plot(sys2Data.results.force, 'r', 'LineWidth', 1.5)
ylabel('Force (N)')
xlabel('Time (s)')
hold off
end
function singleAnalysis(figName,sysName,sysData)
global RTOD
global disturbanceCase

[tEnd, ~] = findSaturation(sysData);
figure('Name', figName)
set(gcf, 'Position', [100 100 800 600])

subplot(5,1,1)
plot(sysData.results.position, 'k', 'LineWidth', 1.5)
ylabel('Position (m)')
xlim([0 tEnd])
legend(sysName)

subplot(5,1,2)
plot(sysData.results.velocity, 'k', 'LineWidth', 1.5)
xlim([0 tEnd])
ylabel('Velocity (m/s)')

subplot(5,1,3)
plot(sysData.results.angle * RTOD, 'k', 'LineWidth', 1.5)
xlim([0 tEnd])
ylabel('Angle (deg)')

if any(disturbanceCase == [2 3 4 5 8 9])
    tSettle = findSettlingTime(sysData)*ones(1,100);
    if isfloat(tSettle)
        hold on 
        plot(tSettle, linspace(-20, 20,100),'--k')
        legend(sysName,'Settling Time');
    end
else
end

subplot(5,1,4)
plot(sysData.results.angleRate * RTOD, 'k', 'LineWidth', 1.5)
xlim([0 tEnd])
ylabel('Angular Rate (deg/s)')

subplot(5,1,5)
plot(sysData.results.force, 'k', 'LineWidth', 1.5)
ylabel('Force (N)')
xlabel('Time (s)')
hold off
end
function [tSat, sampleNum] = findSaturation(results)
    global thetaMax
    theta = results.results.angle.data;
    tOut = results.tout;
    numSamples = length(tOut);
    sampleNum = 1;
    while theta(sampleNum) < thetaMax && sampleNum < numSamples
        sampleNum = sampleNum +1;
    end
    tSat = tOut(sampleNum);
end
function doubleAnimation(figName, sys1Name, sys2Name, sys1Data, sys2Data)
global m1
global m2
global L

sys1Pos = sys1Data.results.position.data;
sys1Phi = sys1Data.results.angle.data + pi();
sys2Pos = sys2Data.results.position.data;
sys2Phi = sys2Data.results.angle.data + pi();
numSamples = length(sys1Pos);
animationFig = figure('Name',figName);
% set(gcf,'
subplot(2,1,1)
plot([-10 10],[0 0],'k','LineWidth',2)
title(sys1Name)
subplot(2,1,2)
plot([-10 10],[0 0],'k','LineWidth',2)
title(sys2Name)

sNum = 1;
while sNum < numSamples && isvalid(animationFig)
    subplot(2,1,1)
    drawcartpend([sys1Pos(sNum), 0, sys1Phi(sNum), 0],m2,m1,L)
    subplot(2,1,2)
    drawcartpend([sys2Pos(sNum), 0, sys2Phi(sNum), 0],m2,m1,L)
    sNum = sNum + 1;
end
end
function singleAnnimation(figName, sysName, sysData)
    global m1
    global m2
    global L
    
    pos = sysData.results.position.data;
    phi = sysData.results.angle.data + pi();
    [~, numSamples] = findSaturation(sysData);
    animationFig = figure('Name',figName);
    plot([-10 10],[0 0],'k','LineWidth',2)
    title(sysName)
    sNum = 1;
    while sNum < numSamples && isvalid(animationFig)
        drawcartpend([pos(sNum), 0, phi(sNum), 0],m2,m1,L)
        sNum = sNum + 1;
    end
end

