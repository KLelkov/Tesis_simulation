function angle = pi2pi(angle)
    while abs(angle) > pi
        angle = angle - sign(angle)*2*pi;
    end
end

