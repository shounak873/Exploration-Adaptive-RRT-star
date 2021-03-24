function [xnear, ynear] = steer(p1,p2)
    step_size = 0.02;
    dist = sqrt((p1(1) - p2(1))^2 + (p1(2) - p2(2))^2);
    dt = step_size;
    theta = atan2(p2(2)-p1(2),p2(1)-p1(1));
    i_comp = dt * cos(theta);
    j_comp = dt * sin(theta);
    
    xnear = p1(1) + i_comp ;
    ynear = p1(2) + j_comp ;
    
    distsamp = norm(p1 - [xnear,ynear]');
    if distsamp < dist 
         return
    else
        xnear = p2(1);
        ynear = p2(2);
    end
    
end