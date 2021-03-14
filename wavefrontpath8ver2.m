function [pathrobot , counter] = wavefrontpath8(cmatrix, sourcex, sourcey, goalx, goaly)

%
% FINDMYPATH8 finds the minimum shortest path from the source to the
% destination by simply checking the 8-Neighbors for the minimum cost and
% then moving to the vertex with the minimum cost.
%
% It takes 5 argumets as an input.
%
% cmatrix = cost matrix for an image to be evaluated for finding the
% shortest path.
%
% sourcex  = x-coordinate for source vertex
% sourcey  = y-coordinate for source vertex
%
%
% goalx  = x-coordinate for goal vertex
% goaly  = y-coordinate for goal vertex
%
%


% Size of the Matrix

[r,c] = size (cmatrix);

% Path of the robot
% pathrobot = zeros ( r,c);

xpath = 2;
ypath = 2;
% Minimum cost
min = 10000;

counter = 0;

% initializing goal coordinates
goalx = goalx;
goaly = goaly;

% initializing start coordinates
cx = sourcex;
cy = sourcey;

currentValue = cmatrix (cx, cy);

% Temporary Values for determining the path of the robot

temp1 = 0 ;
temp2=0;
temp3=0;
temp4 = 0;
temp5 =0;
temp6=0;
temp7= 0 ;
temp8 = 0;

pathrobot(1,1) = cx;
pathrobot(1,2)  =cy;
% Starting from the source to the goal.

while(cmatrix (cx, cy) ~= 2 )
    
    % Calculating 8 - Connected Neighborhood for the source and then
    % determining the minimum cost for the movement of the robot
    
    nx1 = cx-1;
    nx2 = cx;
    nx3 = cx+1;
    nx4 = cx+1;
    nx5 = cx+1;
    nx6 = cx;
    nx7 = cx-1;
    nx8 =cx-1 ;
    
    ny1 = cy-1;
    ny2 = cy-1;
    ny3 = cy-1;
    ny4 = cy;
    ny5 = cy+1;
    ny6 = cy+1;
    ny7 = cy+1;
    ny8 = cy;
    
    %   To check that rows and columns both lie in the boundary of the
    %   matrix
    
    if(( nx1 >= 1 && nx1 <= r) && ( ny1 >=1 && ny1 <= c))
        temp1= cmatrix ( nx1, ny1);
        if ( (temp1 ~= 0) && (temp1 ~= 1)&& (temp1 < currentValue))
            cx = nx1;
            cy = ny1;
            currentValue = temp1;
        end
    end
    
    if(( nx2 >= 1 && nx2 <= r) && ( ny2 >=1 && ny2 <= c))
        temp2= cmatrix ( nx2, ny2);
        
        
        if ( (temp2 ~= 0) && (temp2 ~= 1)&& (temp2 < currentValue))
            
            cx= nx2;
            cy= ny2;
            currentValue = temp2;
        end
    end
    
    
    if(( nx3 >= 1 && nx3 <= r) && ( ny3 >=1 && ny3 <= c))
        temp3= cmatrix ( nx3, ny3);
        
        if ( (temp3 ~= 0) && (temp3 ~= 1) && (temp3 < currentValue))
            cx = nx3;
            cy=ny3;
            currentValue = temp3;
        end
    end
    
    
    if(( nx4 >= 1 && nx4 <= r) && ( ny4 >=1 && ny4 <= c) )
        temp4= cmatrix ( nx4, ny4);
        
        
        if ( (temp4 ~= 0) && (temp4 ~= 1)&& (temp4 < currentValue))
            cx = nx4;
            currentValue = temp4;
        end
        
    end
    
    if(( nx5 >= 1 && nx5 <= r) && ( ny5 >=1 && ny5<= c) )
        
        temp5= cmatrix ( nx5, ny5);
        if ( (temp5 ~= 0) && (temp5 ~= 1)&& (temp5 < currentValue))
            cx = nx5;
            cy = ny5;
            currentValue = temp5;
        end
    end
    
    if(( nx6 >= 1 && nx6 <= r) && ( ny6 >=1 && ny6 <= c) )
        
        temp6= cmatrix ( nx6, ny6);
        if ( (temp6 ~= 0) && (temp6 ~= 1)&& (temp6 < currentValue))
            cx = nx6;
            cy = ny6;
            currentValue = temp6;
            
        end
    end
    
    if(( nx7 >= 1 && nx7 <= r) && ( ny7 >=1 && ny7 <= c) )
        temp7= cmatrix ( nx7, ny7);
        
        if ( (temp7 ~= 0) && (temp7 ~= 1)&& (temp7 < currentValue))
            cx = nx7;
            cy = ny7;
            currentValue = temp7;
        end
    end
    
    if(( nx8>= 1 && nx8 <= r) && ( ny8 >=1 && ny8 <= c) )
        temp8= cmatrix ( nx8, ny8);
        
        if ( (temp8 ~= 0) && (temp8 ~= 1)&& (temp8 < currentValue))
            cx = nx8;
            cy = ny8;
            currentValue = temp8;
        end
    end
        
    pathrobot( xpath,1 ) = cx;
    pathrobot( ypath,2) = cy;
    
    xpath = xpath+1 ;
    ypath = ypath+1;
    
    counter = counter+1;
    
    
end

pathrobot
end
