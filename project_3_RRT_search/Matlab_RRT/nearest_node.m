function [x_near,diff, min_id] = nearest_node(x_rand, T)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
x_near=[];
min_id = 1;
%near_node = [T.v(1).x, T.v(1),y];
diff = 100000.0;
node_count = size(T.v)
for i=1:node_count(2)
    current_dis = sqrt( (T.v(i).x-x_rand(1))^2 + (T.v(i).y-x_rand(2))^2);
    if(current_dis < diff)
        diff = current_dis;
        min_id = i;
    end
end
    
x_near = [T.v(min_id).x, T.v(min_id).y];

end

    