function perturbedSolution = perturbSolution(solution, KpRange, KiRange, KdRange)
    % Randomly perturb each component of the solution within the specified ranges

    % Perturb Kp
    perturbedKp = solution(1) + randInRange(-5, 5); % Adjust the range as desired
    perturbedKp = clamp(perturbedKp, KpRange);

    % Perturb Ki
    perturbedKi = solution(2) + randInRange(-10, 10); % Adjust the range as desired
    perturbedKi = clamp(perturbedKi, KiRange);

    % Perturb Kd
    perturbedKd = solution(3) + randInRange(-2, 2); % Adjust the range as desired
    perturbedKd = clamp(perturbedKd, KdRange);

    % Create the perturbed solution
    perturbedSolution = [perturbedKp, perturbedKi, perturbedKd];
end