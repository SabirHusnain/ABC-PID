% Function to initialize the population of bees
function bees = initializeBees(numBees, KpRange, KiRange, KdRange, G, t, weights)
    bees(numBees).solution = [];
    bees(numBees).fitness = [];
    bees(numBees).trial = [];
    
    for i = 1:numBees
        bees(i).solution = generateRandomSolution(KpRange, KiRange, KdRange);
        bees(i).fitness = evaluateFitness(bees(i).solution, G, t, weights);
        bees(i).trial = 0;
    end
end