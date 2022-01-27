function [result] = angle_lim(angle, lim)
    result = angle;
    if angle > lim(2)
        result = angle - lim(2);
    elseif angle < lim(1)
        result = angle + lim(2);
    end
end