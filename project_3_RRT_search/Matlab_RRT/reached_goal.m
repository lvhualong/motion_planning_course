function is_reached_goal = reached_goal(x_new,x_goal, thro)
%REACHED_GOAL Summary of this function goes here
%   Detailed explanation goes here
    distance = sqrt( (x_new(1)-x_goal(1))^2 + (x_new(2)-x_goal(2))^2);
    if(distance < thro)
        is_reached_goal = 1;
    else
        is_reached_goal = 0;
    end
end

