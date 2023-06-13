clear; clc; close all;

t=0:0.01:5;

% Define the system transfer function
l=0.25; g=9.81; m=0.15; M=0.9;
J=1/3*m*(2*l)^2;
z=M*J+m*M*l^2+m*J;

%Model of Inverted Pendulum
A=[0 1 0 0;
    ((M+m)*m*g*l)/z 0 0 0;
    0 0 0 1;
    -m^2*l^2*g/z 0 0 0];
B=[0; -m*l/z; 0; (J+m*l^2)/z];
C=[1 0 0 0];
D=0;

[num, den]=ss2tf(A,B,C,D);
G=tf(num,den);

% Define the performance criteria and objective function
% In this example, let's consider minimizing settling time, overshoot,
% steady-state error, and input signal
% You can adjust the weights according to the importance of each criterion
weights = [0, 1, 0, 0.1, 1];

% Define the ABC algorithm parameters
maxIterations = 70;  % Maximum number of iterations
numEmployedBees = 50; % Number of employed bees
numOnlookerBees = 50; % Number of onlooker bees
maxTrials = 5;         % Maximum number of trials for a scout bee

% Define the PID controller parameters search ranges
% These ranges may need adjustment depending on the specific problem
KpRange = [-1000, 0];
KiRange = [-2000, 0];
KdRange = [-70, 0];

% Initialize the best solution
bestSolution = [];
bestFitness = Inf;

% Initialize the population of employed bees
employedBees = initializeBees(numEmployedBees, KpRange, KiRange, KdRange, G, t, weights);

% Define the stagnation threshold
stagnationThreshold = 5; % Number of iterations without improvement to detect stagnation

% Initialize the stagnation counter
stagnationCounter = 0;

% Perform the ABC algorithm iterations
for iteration = 1:maxIterations
    % Employed bees phase
    for i = 1:numEmployedBees
        % Generate a neighbor solution
        neighborSolution = generateNeighborSolution(employedBees(i).solution, KpRange, KiRange, KdRange);

        % Evaluate the neighbor solution fitness
        employedBees(i).neighborSolution = neighborSolution;
        employedBees(i).neighborFitness = evaluateFitness(neighborSolution, G, t, weights);

        % Compare the neighbor solution with the current solution
        if employedBees(i).neighborFitness < employedBees(i).fitness
            employedBees(i).solution = neighborSolution;
            employedBees(i).fitness = employedBees(i).neighborFitness;
            employedBees(i).trial = 0; % Reset the trial counter

            % Reset the stagnation counter
            % stagnationCounter = 0;
        else
            employedBees(i).trial = employedBees(i).trial + 1; % Increment the trial counter
        end
    end

    % Onlooker bees phase
    onlookerBees = selectOnlookerBees(employedBees, numOnlookerBees);

    for i = 1:numOnlookerBees
        % Generate a neighbor solution
        neighborSolution = generateNeighborSolution(onlookerBees(i).solution, KpRange, KiRange, KdRange);

        % Evaluate the neighbor solution fitness
        onlookerBees(i).neighborSolution = neighborSolution;
        onlookerBees(i).neighborFitness = evaluateFitness(neighborSolution, G, t, weights);

        % Compare the neighbor solution with the current solution
        if onlookerBees(i).neighborFitness < onlookerBees(i).fitness
            onlookerBees(i).solution = neighborSolution;
            onlookerBees(i).fitness = onlookerBees(i).neighborFitness;
            onlookerBees(i).trial = 0; % Reset the trial counter

            % Reset the stagnation counter
            % stagnationCounter = 0;
        else
            onlookerBees(i).trial = onlookerBees(i).trial + 1; % Increment the trial counter
        end
    end

    % Scout bees phase
    for i = 1:numEmployedBees
        if employedBees(i).trial >= maxTrials
            % Generate a new random solution for the scout bee
            employedBees(i).solution = generateRandomSolution(KpRange, KiRange, KdRange);
            employedBees(i).fitness = evaluateFitness(employedBees(i).solution, G, t, weights);
            employedBees(i).trial = 0; % Reset the trial counter
        end
    end

    % Memorize the best solution
    for i = 1:numEmployedBees
        if employedBees(i).fitness < bestFitness
            bestSolution = employedBees(i).solution;
            bestFitness = employedBees(i).fitness;
        end
    end

    % Check for stagnation
    if iteration > 1 && bestFitness == prevBestFitness
        stagnationCounter = stagnationCounter + 1;
    else
        stagnationCounter = 0;
    end

    % If stagnation is detected, perturb the best solution
    if stagnationCounter >= stagnationThreshold
        bestSolution = perturbSolution(bestSolution, KpRange, KiRange, KdRange);
        bestFitness = evaluateFitness(bestSolution, G, t, weights);
        stagnationCounter = 0; % Reset the stagnation counter
    end

    % Store the current best fitness for comparison in the next iteration
    prevBestFitness = bestFitness;

    % Display the best solution in each iteration
    disp(['Iteration ', num2str(iteration), ': Best Solution = [Kp = ', num2str(bestSolution(1)), ...
        ', Ki = ', num2str(bestSolution(2)), ', Kd = ', num2str(bestSolution(3)), ...
        '], Best Fitness = ', num2str(bestFitness)]);

    bobSolution(iteration,1:3)=bestSolution;
    bobFitness(iteration)=bestFitness;
end

% Apply the optimized PID controller to the system
Kp=0; Ki=0; Kd=0;
while(Kp==0 && Ki==0 && Kd==0)
    [minFit, itr]=min(bobFitness);
    Kp = bobSolution(itr,1);
    Ki = bobSolution(itr,2);
    Kd = bobSolution(itr,3);
    if(Kp==0 && Ki==0 && Kd==0)
        bobFitness(itr)=Inf;
    end
end

controller = pid(Kp, Ki, Kd, 0.03);
sysClosedLoop = feedback(controller * G, 1);
sysClosedLoopInput = feedback(controller, G);

figure(1);
yout=step(sysClosedLoop, t);
plot(t,yout,'-r', 'LineWidth', 2);
title('Step Response of the System \theta(t)');
xlabel('time (s)');
ylabel('angle (rad)');
grid on; grid minor;

figure(2);
uout=step(sysClosedLoopInput, t);
plot(t,uout*0.01,'-k', 'LineWidth', 2); % 0.01 is radiu, T=rF
title('Input Signal of Plant u(t)');
xlabel('time (s)');
ylabel('magnitude');
grid on; grid minor;

figure(3)
plot(bobFitness,'-r')
title('Fitness of ABC')
xlabel('iterations')
ylabel('fitness')
grid on; grid minor

save('optimalValues.mat','Kp','Ki','Kd');
disp('Optimal Solution Saved!');