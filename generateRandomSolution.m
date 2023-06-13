% Function to generate a random solution
function solution = generateRandomSolution(KpRange, KiRange, KdRange)
    Kp = rand() * (KpRange(2) - KpRange(1)) + KpRange(1);
    Ki = rand() * (KiRange(2) - KiRange(1)) + KiRange(1);
    Kd = rand() * (KdRange(2) - KdRange(1)) + KdRange(1);
    solution = [Kp, Ki, Kd];
end