% Function to select onlooker bees based on fitness values
function onlookerBees = selectOnlookerBees(employedBees, numOnlookerBees)
    fitnessValues = [employedBees.fitness];
    probabilities = fitnessValues / sum(fitnessValues);
    
    indices = randsample(1:length(employedBees), numOnlookerBees, true, probabilities);
    onlookerBees = employedBees(indices);
end