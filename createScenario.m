function [sysDisturbance, sysSetpoints] = createScenario(caseNum,dt, tEnd, params)
global x0
velocityLimit = params(1);
angleDotLimit = params(2);
chanceOfDisturb = params(3);


numSamples = tEnd/dt;
t(1:numSamples,1) = dt:dt:tEnd;
states = zeros(numSamples,4);
setpoints = zeros(numSamples,4);
disturbData = [t states];
setpointsData = [t setpoints];
x0 = [0 0 0 0];

switch caseNum
    case 1 % no disturbance
        x0 = [0 0 1 0];
   
    case 2  % step of 1 m cart position
            setpointsData(1/dt:end,2) = 1;
                
    case 3  % ramp of 1 m/s 
        for sampleNum = 1/dt:numSamples
            setpointsData(sampleNum,2) = sampleNum*dt -1;
        end
        
    case 4  % impulse of 1 m/s cart velocity
        disturbData(1/dt,3) = 1;
    
    case 5  % impulse of 1 rad/s arm angular velocity
        disturbData(1/dt,5) = 1;
    
    case 6  % repeated impulse of 1 m/s velocity
        for sampleNum = 1/dt:numSamples
            if checkTime(disturbData(sampleNum,1))
                disturbData(sampleNum,3) = 1;
            end
        end
        
    case 7  % repeated impulse of 1 rad/s arm angular velocity
        for sampleNum = 1/dt:numSamples
            if checkTime(disturbData(sampleNum,1))
                disturbData(sampleNum,3) = rand * angleDotLimit * pi()/180;
            end
        end
        
    case 8  % ramp of 1 m/s^2 cart velocity
        for sampleNum = 1/dt:numSamples
            disturbData(sampleNum,2) = sampleNum * dt;
        end
        
    case 9  % ramp of 1 rad/s^2 arm angular velocity
        for sampleNum = 1/dt:numSamples
            disturbData(sampleNum,3) = sampleNum * dt;
        end
        
    case 10 % random disturbance
        for sampleNum = 1/dt:numSamples
            randVel = rand;
            randAngDot = rand;
            if randVel < chanceOfDisturb
                disturbData(sampleNum,3) = rand * velocityLimit;
            end
            if randAngDot < chanceOfDisturb
                disturbData(sampleNum,3) = rand * velocityLimit * pi()/180;
            end
        end
end
sysDisturbance = timeseries(disturbData(:,2:end),disturbData(:,1));
sysSetpoints = timeseries(setpointsData(:,2:end),setpointsData(:,1));
end

function isTrue = checkTime(time)
    if time == floor(time)
        isTrue = true;
    else
        isTrue = false;
    end
end