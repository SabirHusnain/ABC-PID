function value = randInRange(minValue, maxValue)
    % Generate a random value within the specified range
    value = minValue + (maxValue - minValue) * rand();
end