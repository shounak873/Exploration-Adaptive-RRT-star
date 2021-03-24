function [isInside, dist] = ellipsd(foc1,foc2,a,point)
    dist = norm(point - foc1) + norm(point - foc2);
    if dist <= 2*a
        isInside = true;
    else
        isInside = false;
    end
end