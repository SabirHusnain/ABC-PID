function clampedValue = clamp(value, range)
    % Clamp the value to the specified range
    clampedValue = max(min(value, range(2)), range(1));
end
