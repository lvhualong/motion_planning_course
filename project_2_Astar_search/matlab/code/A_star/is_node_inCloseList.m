function  node_in_closeList = is_node_inCloseList(CLOSE,node)
%IS_OPENLIST_EMPTY Summary of this function goes here
%   Detailed explanation goes here
    node_in_closeList = 0; 
    count = size(CLOSE,1);
    for i=1:count
        if(CLOSE(i,1)==node(1) && CLOSE(i,2)==node(2))
           node_in_closeList = 1; 
           break
        end
    end
    
end

