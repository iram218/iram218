
function [PointsofInterest, pathSum] = findInflectionPoint( P )

% finfInflectionPoint is a procedure that takes the linear piecewise path 
% (formed as a result of applying wavefront method) and determines the
% inflection points.
% Curvature of a straight line is zero.
% We use this assumption to detect the points where curvature changes. 
% The points are marked as true inflection points
% and are stored in 'PointsofInterest' matrix for further processing.

[r c] = size(P);
Curvature = zeros(r-2,1);

count = 0;

% Calculating Curvature of the path points.

for i = 1:r-2
    
    x1 = P(i,1);
    x2 = P(i+1,1);
    x3 = P(i+2,1);
    
    y1 = P(i,2);
    y2 = P(i+1,2);
    y3 = P(i+2,2);
    
    K = 2 * ((( x2 - x1)*( y3-y2))- (( y2-y1)*(x3-x2)));
    
    K = K / sqrt( ((x2-x1)^2 +( y2-y1)^2)* ((x3-x2)^2 + (y3-y1)^2) * ((x1-x2)^2+(y1-y3)^2));
    
    Curvature(i) = K ;
    
    if K ~= 0
        count = count+1;
    end
end
%

% Determining the way-points where curvature is non-zero indicating
% inflection.

PointsofInterest = zeros(count+2,2);
[ro1 col] = size(PointsofInterest);
X1 = P(1,1);
Y1 = P(1,2);

X2 = P(r,1);
Y2 = P(r,2);
PointsofInterest(1,1) = X1;
PointsofInterest(1,2) = Y1;
PointsofInterest(count+2,1)= X2;
PointsofInterest(count+2,2)= Y2;

size( PointsofInterest);
x = 2;
y = 2;

for index = 1: r-2
    
    value = Curvature(index);
    
    if value ~= 0
        
        X = P(index+1,1);
        Y = P(index+1,2);
        
        PointsofInterest(x,1)= X;
        PointsofInterest(x,2)= Y;
        
        x = x+1;
        
        
    end
    
    
    
end
pathSum = 0;
[rows cols] = size(PointsofInterest);
for i = 1 : rows-1
    x1 = PointsofInterest(i,1);
    y1 = PointsofInterest(i,2);

    x2 = PointsofInterest(i+1,1);
    y2 = PointsofInterest(i+1,2);

    pathSum = pathSum + ( ( sqrt( ( x2 - x1) .^ 2 ) + ( ( y2 - y1) .^ 2 ) ) );
    

end

end







