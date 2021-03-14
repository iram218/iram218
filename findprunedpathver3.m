function [ prunedpath ] = findprunedpathver3( inflectionPoints, fImage)

disp('Inside find pruned path ver3 ');
flag =0 ;
[r c ]= size (inflectionPoints);

finalImage = fImage;
xpath = 1;
indexofpoints = 1;

% For finding pruned path there must be atleast 3 inflection points.

prunedpath(1,1) = inflectionPoints (r , 1);
prunedpath(1,2) = inflectionPoints (r, 2);

 endx = inflectionPoints( r, 1);
 endy = inflectionPoints (r, 2);
 xstart = inflectionPoints( 1, 1);
 ystart = inflectionPoints (1, 2);


while( (xstart ~= endx) || (ystart ~= endy))
  
% %    pause(3) 
%     flag = 0;
   
for t = 0:0.001: 1
        
        X1 = floor(xstart*(1-t) + endx * t);
        Y1 = floor(ystart*(1-t) + endy * t);
        X = ceil(xstart*(1-t) + endx * t);
        Y =ceil(ystart*(1-t) + endy * t);
        
        temp = finalImage( X,Y);
        temp1 = finalImage (X1,Y1);
        % check if the new point resides on obstacle
        
        if( (temp == 65535) && (temp1 == 65535))
            flag = 1;
        end
    end
    % if obtacle found on joing end an start coordinates via straight line
   if ( flag == 0)
        indexofpoints = 1;
        endx = xstart;
        endy = ystart;
        xstart = inflectionPoints(indexofpoints,1);
        ystart = inflectionPoints(indexofpoints,2);
        xpath = xpath+1;
        prunedpath( xpath,1) = endx;
        prunedpath( xpath,2) = endy;
    end
   
    if (flag == 1)
       
        disp('flag==1');
        indexofpoints = indexofpoints + 1;
        xstart = inflectionPoints(indexofpoints,1);
        ystart = inflectionPoints(indexofpoints,2);
        flag = 0;        
    end
     
    
end


end


