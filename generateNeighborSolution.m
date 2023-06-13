% Function to generate a neighbor solution
function neighborSolution = generateNeighborSolution(solution, KpRange, KiRange, KdRange)
    neighborSolution = solution;
    for i = 1:length(solution)
        % Modify each parameter by adding/subtracting a small random value
        neighborSolution(i) = neighborSolution(i) + (rand() - 0.5) * 0.2 * (KpRange(2) - KpRange(1));
        
        % Ensure the new value is within the search range
        neighborSolution(i) = max(min(neighborSolution(i), KpRange(2)), KpRange(1));
    end
end
