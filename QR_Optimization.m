% [Q,R] = QR_Optimization(Q_initial,R_initial)
% Description:
% This function takes estimated optimum cost weights and returns Q and R
% matricies that have been optimized using matlabs "fminsearch" function.

function [Q, R] = QR_Optimization(initWeights, A, B)

% Define Variables
global Astar
global Bstar
global R
Astar = A;
Bstar = B;
x0 = initWeights;
options = optimset('PlotFcns',@optimplotfval,'Tolx',1.0,'Display','iter');

weights = fminsearch(@simFun,x0,options);
for varNum = 1:length(weights)
    if weights(varNum) < 0.1
        weights(varNum) = 0.1;
    end
end

Q = diag(weights(1:4));
end

function error = simFun(x0)
global K
global sys
global R
global SIMULATION_NAME
global dt

formCost = 0;
for varNum = 1:length(x0)
    if x0(varNum) <= 0
        x0(varNum) = 0.0001;
        formCost = formCost + 300000*abs(x0(varNum));
    end
end

% Format Inputs
Q = diag(x0(1:4));

K = lqr(sys,Q,R);
results = sim(SIMULATION_NAME);
tSettle = findSettlingTime(results);
stateError = sum(sum(abs(results.results.error.data))); 
error = stateError;
end