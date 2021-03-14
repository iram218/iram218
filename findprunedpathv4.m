function [ prunedpath, pruneSum ] = findprunedpathv4( inflectionPoints, fImage)

disp('Inside find pruned path v4');
[r c ]= size (inflectionPoints);
xpath = 1;
% For finding pruned path there must be atleast 3 inflection points.
prunedpath(1,1) = inflectionPoints (r , 1);
prunedpath(1,2) = inflectionPoints (r, 2);
pruneSum = 0;
while ( r > 1 )
    
    minIndex = r;    
    xstart = inflectionPoints (r,1);
    ystart = inflectionPoints (r,2);
    
    for i = r-1 :-1: 1
        
        endx = inflectionPoints( i, 1);
        endy = inflectionPoints (i, 2);        
        flag = 0;
        
        for t = 0:0.001: 1
            
            X1 = floor(xstart*(1-t) + endx * t);
            Y1 = floor(ystart*(1-t) + endy * t);
            X = ceil(xstart*(1-t) + endx * t);
            Y =ceil(ystart*(1-t) + endy * t);            
            temp = fImage( X,Y);
            temp1 = fImage (X1,Y1);
            % check if the new point resides on obstacle            
            %if( (temp == 65535) || (temp1 == 65535))
            if( (temp == 65535) || (temp1 == 65535))
                flag = 1;      break;
            end
        end
        if (flag == 0 )            
            minIndex = i;            
        end
        
    end
    
    if (minIndex == r)
        xpath = xpath + 1;
        r = r - 1;
        prunedpath(xpath,1) = inflectionPoints (r,1);
        prunedpath(xpath,2)= inflectionPoints (r,2);
    else
        r = minIndex;
        xpath = xpath+ 1;
        prunedpath(xpath,1) = inflectionPoints (r,1);
        prunedpath(xpath,2)= inflectionPoints (r,2);
        
    end
    
    prunedpath
end
pathSum = 0;
[rows cols] = size(prunedpath);
for i = 1 : rows-1
    x1 = prunedpath(i,1);
    y1 = prunedpath(i,2);

    x2 = prunedpath(i+1,1);
    y2 = prunedpath(i+1,2);

   % pruneSum = pruneSum + ( ( sqrt( ( x2 - x1).^ 2 ) + ( ( y2 - y1).^ 2 ) ) );
      pathSum = pathSum + sqrt( ((x2-x1).^2 )+ ((y2-y1).^2 ));

end
pathSum
pruneSum = pathSum;
end

