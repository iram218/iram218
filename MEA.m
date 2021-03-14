
function [c] = MEA( imageMatrix , sx, sy, gx, gy)

% CREATECOSTMATRIX creates the cost matrix for the image to be using by
% dijkstra's algorithm in finding the minimum cost path from source to
% destination
% It takes three arguments as an input
%
%  imageMatrix = Binary Image in which 0 represents open vertices and 1
%   represents obstacles within an image.
%
%   gx = x-xoordinate  of the goal
%   gy = y-coordinate of the goal
%
%
% The intended purpose for this matrix is that we start from our goal and
% we assign an intiial value say '2' to represent it as a goal.
% Then we look for its 8- neighbours and assign them a value which is
% exactly one greater than the previous value .
%
%
%           x - 1, y-1 |  x -1 , y | x-1, y+1
%          -----------------------------------
%           x , y-1    |  x, y     | x, y+1
%         -------------------------------------
%           x + 1 ,y-1 | x +1 , y  | x+1, y+1
%
%   prevcost = prevcost + 1
%
% We iterate for each pixel finds its neighbors and assign them a value as
% mentioned above.
%

disp (' Initializing Values')
% Total rows and total columns.

[row, cols ] = size(imageMatrix);

% CostMatrix that is initialized to zero.

% c = zeros ( row, cols);
c = imageMatrix;
% Temporary variables used for keeping the initial cost.

% Setting the goal cost.
c(gx,gy) = 2;
% setting the Mainhattton Distance
%MD = sqrt((sx-gx)*(sx-gx)+(sy-gy)*(sy-gy) );
% start iteration from the goal point.

i = gx ;
j = gy ;

% source coordinates
sourcex = sx;
sourcey = sy;
% Previous Cost intially set to '2'

prevcost = 2;


% Iterate the whole image for determining the cost

disp( '--------   Iteration Start in MEA*  ----------- ');

 while( (sourcex ~= i) || (sourcey ~= j)  ) 
   
    
            % if pixel value == previous cost value then find the
            % 8- neighbours for the pixel.
            
                     
                %calculate 8 neighbours
                
                nx1 = i -1;
                ny1 = j-1;
                
                nx2 = i;
                ny2 = j-1;
                
                nx3 = i+1;
                ny3 = j-1;
                
                nx4 = i+1;
                ny4 = j ;
                
                nx5 = i+1;
                ny5 = j+1;
                
                nx6 = i;
                ny6 = j+1;
                
                nx7 = i-1;
                ny7 = j+1;
                
                nx8= i-1;
                ny8= j;
                
                flag1 = 0;
                flag2 = 0;
                flag3=0;
                flag4 = 0;
                flag5 = 0;
                flag6 = 0 ;
                flag7 = 0 ;
                flag8 = 0;
                
                  ecost = [ 1000 1000 1000 1000 1000 1000 1000 1000 ];                  
                
                
                % If the pixel values lies in the boundary of the image
                % and they are not obstacles then set the cost of the
                % neighboringpixel.
                
                % Neighbor1
                if( ( nx1 <= row )&& ( ny1 <= cols))
                    if(( nx1 >=1) && (ny1 >=1))
                        if( c( nx1, ny1) == 0)
                            hn = sqrt((sx-nx1)*(sx-nx1)+(sy-ny1)*(sy-ny1) );
                            gn = 14;
                            ecost(1) = hn + gn;                            
%                             pause(1)
                        end
                    end
                end
                
                %Neighbor2                
                if(( nx2 <= row ) && ( ny2 <= cols))
                    flag2 = 1;
                    if( (nx2 >=1) && ( ny2 >=1 ))
                        if( c(nx2, ny2) == 0)
                            hn = sqrt((sx-nx2)*(sx-nx2)+(sy-ny2)*(sy-ny2) );
                            gn = 10;
                            ecost(2) =  hn + gn;
%                             pause(1)
                        end
                    end
                end
                
                % Neighbor 3
                if( ( nx3 <= row ) && ( ny3 <= cols))
                    flag3 = 1;
                    if(( nx3 >=1) && ( ny3 >=1))
                        if(c( nx3, ny3) == 0)                            
                            hn = sqrt((sx-nx3)*(sx-nx3)+(sy-ny3)*(sy-ny3) );
                            gn = 14;
                            ecost(3) = hn + gn;
%                             pause(1)
                        end
                    end
                end
                
                % Neighbor 4
                
                if( ( nx4 <= row ) && ( ny4 <= cols))
                    flag4 = 1;
                    if (( nx4 >=1) && (ny4 >=1))
                        if( c( nx4, ny4) == 0)
                            hn = sqrt((sx-nx4)*(sx-nx4)+(sy-ny4)*(sy-ny4) );  
                            gn = 10;
                            ecost(4) = hn + gn;
%                             pause(1)
                        end
                    end
                end
                
                % Neighbor 5
                
                if( ( nx5 <= row ) && ( ny5 <= cols))
                    flag5 = 1;
                    if (( nx5 >=1) && (ny5 >=1))
                        if( c( nx5, ny5) == 0)                            
                           hn = sqrt((sx-nx5)*(sx-nx5)+(sy-ny5)*(sy-ny5) );
                           gn = 14;
                           ecost(5) = hn + gn;
%                             pause(1)
                        end
                    end
                end
                
                % Neighbor 6
                
                if( ( nx6 <= row ) && ( ny6 <= cols))
                    flag6 = 1;
                    if ((nx6 >=1 ) && ( ny6 >=1))
                        if( c( nx6, ny6) == 0)                            
                           hn = sqrt((sx-nx6)*(sx-nx6)+(sy-ny6)*(sy-ny6) );
                           gn = 10;
                           ecost(6) = hn + gn;
%                             pause(1)
                        end
                    end
                    
                end
                
                % Neighbor 7
                
                if( ( nx7 <= row ) && ( ny7 <= cols))
                    flag7 = 1;
                    if(( nx7 >=1 ) && (ny7 >=1))
                        if( c( nx7, ny7) == 0)                            
                            hn = sqrt((sx-nx7)*(sx-nx7)+(sy-ny7)*(sy-ny7) );
                            gn = 14;
                            ecost(7) = gn + hn;
%                             pause(1)
                      
                        end
                    end
                end
                
                % Neighbor 8
                
                if( ( nx8 <= row ) && ( ny8 <= cols))
                    flag8 = 1;
                    if(( nx8 >=1) && ( ny8 >=1 ))
                        if( c( nx8, ny8) == 0)                            
                            hn = sqrt((sx-nx8)*(sx-nx8)+(sy-ny8)*(sy-ny8) );
                            gn = 10;
                            ecost(8) = gn + hn;
                        end
                    end
                end
%                 for i = 1 : 8
%                     if ecost(i) > MD
%                         ecost(i) = ecost(i) + (MD-ecost(i));
%                     end
%                 end
                
                %finding minimum cost with in 8 nrighbours
%                 ecost
%                 disp('Minimum E-Cost')
                minecost = min(ecost);
%                 pause(5);
                
                %finding index of the minimum cost
                
                indexofecost = find (ecost == minecost)  ;
                
                % NOW SETTING COST TO MIN E-COST
                
                
                if(indexofecost(1) == 1 )
                    c(nx1, ny1) = prevcost +1;
                    i = nx1 ;
                    j = ny1 ;
                end
                
                
                
                if(indexofecost(1) == 2)
                    c(nx2, ny2) = prevcost +1;
                     i = nx2 ;
                    j = ny2 ;
                end
                
                
                if (indexofecost(1) == 3)
                    c(nx3, ny3) = prevcost +1;
                     i = nx3 ;
                    j = ny3 ;
                end
                
                
                if (indexofecost(1) == 4)
                    c(nx4, ny4) = prevcost +1;
                     i = nx4 ;
                    j = ny4 ;
                end
                
                
                if (indexofecost(1) == 5)
                    c(nx5, ny5) = prevcost +1;
                     i = nx5 ;
                    j = ny5 ;
                end
                
                
                if (indexofecost(1) == 6)
                    c(nx6, ny6) = prevcost +1;
                     i = nx6 ;
                    j = ny6 ;
                end
                
                
                if (indexofecost(1) == 7)
                    c(nx7, ny7) = prevcost +1;
                     i = nx7 ;
                    j = ny7;
                end
                
                
                if (indexofecost(1) == 8)
                    c(nx8, ny8) = prevcost +1;
                     i = nx8 ;
                    j = ny8;
                end
       
    prevcost = prevcost + 1;
 end % while loop
end % End of Function