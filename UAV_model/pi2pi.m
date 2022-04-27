function angle = pi2pi(angle)
    if abs(angle) > 1e+2
        throw(MException("pi2pi:exception", "Angle is unexpectingly large..."));
    end
    while abs(angle) > pi
        angle = angle - sign(angle)*2*pi;
    end
end

