% Function to evaluate the fitness of a solution
function fitness = evaluateFitness(solution, G, t, weights)
    Kp = solution(1);
    Ki = solution(2);
    Kd = solution(3);
    
    controller = pid(Kp, Ki, Kd, 0.01);
    sysClosedLoop = feedback(controller * G, 1);
    sysClosedLoopInput = feedback(controller, G);
    y = step(sysClosedLoop, t);
    U = step(sysClosedLoopInput, t);
    
    if y(end)>=0.98
        idxTime=find(y > 0.98, 1);
        settlingTime = t(idxTime);
    else
        settlingTime=t(end);
    end
    overshoot = max(y) - 1;
    steadyStateError = abs(1-y(end));
    inputSignal = max(abs(U));
    % inputSignal=0;
    error=sum(abs(t'.*(1-y)));
    % error=0;
    
    fitness = weights(1) * settlingTime + weights(2) * overshoot + ...
        weights(3) * steadyStateError + weights(4) * inputSignal+ ...
        weights(5)*error;

    if isnan(fitness) || isinf(fitness)
        fitness=6.02*10^23;
    end
end
