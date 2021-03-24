function factor = escale(foc1,foc2,a,point)
    [isinside, dist] = ellipsd(foc1,foc2,a,point);
    if isinside == false
        factor = 0.001;
    else
        factor = 1;
    end
end