function tSettle = findSettlingTime(results)
%define variables
global disturbanceCase
isFinished = false;
i = 1;
dt = results.tout(2) - results.tout(1);
position = results.results.position.data;
velocity = results.results.velocity.data;

SSposition = position(end);
SSvelocity = velocity(end);

while ~isFinished && i ~= length(results.tout)
    if results.tout(i) < 1.1    % continue if step has already occured
        isFinished = false;
    else
        tSettle = results.tout(i);
        dv = (velocity(i) - velocity(i - 1))/dt;
    end
    if disturbanceCase ==3  && abs(velocity(i) - SSvelocity) < 0.02  && abs(dv) < .02
        isFinished = true;
    else
        if abs(position(i) - SSposition) < .02 * abs(SSposition) &&...
                abs(velocity(i)) < .05
            isFinished = true;
        end
        i = i+1;
    end
    if i == length(results.tout)
        tSettle = false;
    end
end


