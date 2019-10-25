function path = A_star_search(map,MAX_X,MAX_Y)
%%
%This part is about map/obstacle/and other settings
    %pre-process the grid map, add offset
    size_map = size(map,1);
    disp(size_map);
    Y_offset = 0;
    X_offset = 0;
    
    %Define the 2D grid map array.
    %Obstacle=-1, Target = 0, Start=1
    MAP=2*(ones(MAX_X,MAX_Y));
    
    %Initialize MAP with location of the target
    xval=floor(map(size_map, 1)) + X_offset;
    yval=floor(map(size_map, 2)) + Y_offset;
    xTarget=xval;
    yTarget=yval;
    MAP(xval,yval)=0; %target is 0
    disp(size(MAP,1))%10*10
    
    %Initialize MAP with location of the obstacle
    for i = 2: size_map-1
        xval=floor(map(i, 1)) + X_offset;
        yval=floor(map(i, 2)) + Y_offset;
        MAP(xval,yval)=-1;%obstal is -1
    end 
    
    %Initialize MAP with location of the start point
    xval=floor(map(1, 1)) + X_offset;
    yval=floor(map(1, 2)) + Y_offset;
    xStart=xval;
    yStart=yval;
    MAP(xval,yval)=1;%start is 1

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %LISTS USED FOR ALGORITHM
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %OPEN LIST STRUCTURE
    %--------------------------------------------------------------------------
    %IS ON LIST 1/0 |X val |Y val |Parent X val |Parent Y val |h(n) |g(n)|f(n)|
    %--------------------------------------------------------------------------
    OPEN=[];
    %CLOSED LIST STRUCTURE
    %--------------
    %X val | Y val |
    %--------------
    % CLOSED=zeros(MAX_VAL,2);
    CLOSED=[];

    %Put all obstacles on the Closed list
    k=1;%Dummy counter
    for i=1:MAX_X
        for j=1:MAX_Y
            if(MAP(i,j) == -1)
                CLOSED(k,1)=i;
                CLOSED(k,2)=j;
                k=k+1;
            end
        end
    end
    CLOSED_COUNT=size(CLOSED,1);
    %set the starting node as the first node
    xNode=xval;
    yNode=yval;
    
    OPEN_COUNT=1;
    goal_distance=distance(xNode,yNode,xTarget,yTarget);
    path_cost=0;
    disp(goal_distance)
    OPEN(OPEN_COUNT,:)=insert_open(xNode,yNode,xNode,yNode,goal_distance,path_cost,goal_distance);
    
    NoPath=1;%flag no path found
 

%%
%This part is your homework
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% START ALGORITHM
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %while(is_openlist_empty(OPEN) ~= 1)
    while(min_fn(OPEN,OPEN_COUNT,xTarget,yTarget) ~= -1)
        i_min = min_fn(OPEN,OPEN_COUNT,xTarget,yTarget);
        
        %pop the min_fn_node from OpenList, add to Closed list
        current_node = OPEN(i_min,:);
        OPEN(i_min, 1)=0;        
        CLOSED_COUNT=CLOSED_COUNT+1;
        CLOSED(CLOSED_COUNT,1)=current_node(2);
        CLOSED(CLOSED_COUNT,2)=current_node(3);
        
        %fprintf('the min node(%d, %d)\n',current_node(2),current_node(3));
        % check the current is the goal?
        if(current_node(2)==xTarget && current_node(3)==yTarget)
            disp("find the goal");
            NoPath = 0;
            break;
        end
        
        %expand the current to the neighbors
        neighborPtrSets = expand_array(current_node(2),current_node(3),current_node(7),xTarget,yTarget,CLOSED,MAX_X,MAX_Y);
        neighbor_size = size(neighborPtrSets,1);
        for i=1:neighbor_size
            neighbor_node = neighborPtrSets(i,:);
            
            %check the node in obstacle or closedList,忽略掉
            if(is_node_inCloseList(CLOSED,neighbor_node) == 1)
                continue;
            end
            
            %check the node in the OpenList,更新ｇ(n)and parent
            n_index = node_index(OPEN,neighbor_node(1),neighbor_node(2));
            if(OPEN(n_index,1)==1)
                if(neighbor_node(4)<OPEN(n_index,7))
                    OPEN(n_index,4) = current_node(2);
                    OPEN(n_index,5) = current_node(3);
                    OPEN(n_index,7) = neighbor_node(4);  
                    continue;
                end
            end
            
            %expand new node
            OPEN_COUNT = OPEN_COUNT+1;
            OPEN(OPEN_COUNT,:)=insert_open(neighbor_node(1),neighbor_node(2),current_node(2),current_node(3),neighbor_node(3),neighbor_node(4),neighbor_node(5));      
                  
        end
        
    end
  
    path=[];
    if(NoPath==0)
        path_count = 1;
        temp_node=OPEN(node_index(OPEN,xTarget,yTarget),:);
        while(temp_node(2) ~=xStart && temp_node(3) ~=yStart)
            path(path_count,:)=[temp_node(2)-0.5, temp_node(3)-0.5];
            fprintf('the path node(%d, %d)\n',path(path_count,1),path(path_count,2));
            path_count = path_count+1;
            temp_node=OPEN(node_index(OPEN,temp_node(4),temp_node(5)),:);
        end  
        path(path_count,:)=[temp_node(2), temp_node(3)];
%         fprintf('the path node(%d, %d)\n',path(path_count,1),path(path_count,2));
%         path_count = path_count+1;
%         path(path_count,:)=[xStart,yStart];
        fprintf('the path node(%d, %d)\n',path(path_count,1),path(path_count,2));
        disp(size(path,1))
    else
            disp("can not find the path");
    end
end
