
function [] = MainProject()
%
% Main Project to run the program
%
% Path of image folder is already set, images naming convention is set as image 1, image2 and so on,
% you can set image no from 1 to 10 in line no 12 in source code

currentfolder = pwd ;

% provide image number of input image file in the following, default is 1
%fileName = strcat(currentfolder ,'\InputImages\image', num2str('0'),'.jpg');
% provide image number of input image file in the following, default is 1

mapName = 'M5'; % set file name of input image grid map i.e. occupancy grid
fileName = strcat(currentfolder ,'\InputImages\', mapName ,'.jpg')

rgbI = imread(fileName);

% Convert it to Gray Scale Image
I = rgb2gray (rgbI);
[w , h ] = size(I) % get width and height of envirnment map

% take start and end position from user
figure('Name', 'Original', 'NumberTitle','off'), imagesc(1:h, 1:w, rgbI),title ('Select two points from the figure');
% colormap gray;
hold on
% Capturing input from the user for source and destination points
[y,x] = ginput (2);
% % Seperating the both points from the vector [x,y] for source s and
% destination d
xs = x(1);
ys = y(1);

xs = ceil ( xs )
ys = ceil ( ys)
xd = ceil ( x(2))
yd = ceil ( y(2))

P =[ xs ys ; xd yd];
scatter(P(:,2), P(:,1),'filled','ro')

% % colormap gray
hold off
% Check whether the source or destination lies on an obstacle if not then
% apply the remaining path finding procedure.
% Calling CREATECOSTMATRIX

if ( xs == xd && ys == yd )
    disp('invalid data : same start and end positions, program terminated');
    close;
else
    % pre allocate memory for cost matrix and image matrix initialized with zero
    
    CostMatrixofImageModified = ones( w,h);
    CostMatrixofImage = ones( w,h);
    %%%%%%%%%% image processing code starts
    % apply smoothing filter to decrease noise if needed
    %H = fspecial ('average', 5);
    %I = imfilter ( I, H );
    
    % Find the Gray Threshold Level using otsu
    level = graythresh( I );
    % Convert Image to Binary Image
    binaryImage = im2bw( I, level);
    % removes all connected components less than 30 pixels
    binaryImage = bwareaopen(binaryImage,30);
    % fill gaps using closing
    se = strel('square',1);
    binaryImage = imclose(binaryImage,se);
    
    % set obstacles to black color
    finalImage = binaryImage;
    finalImage = im2uint16(finalImage);
    finalImage = imcomplement(binaryImage); % optional paramter setting
    %    figure ('Name', 'BinaryImage', 'NumberTitle','off') , imshow( finalImage ) ,title('Binary Image');
    
    %%%%%%%%% image processing code ends
    
    if ( (finalImage (xs, ys ) == 0 ) && ( finalImage (xd, yd ) == 0))            
        
        % Dilate Image to create the Configuration space for the robot.
        
        doubleImage = ones(w,h);
        doubleImage = im2double ( finalImage);
        sedilate = strel('square',1);
        dilateImage = imdilate (doubleImage, sedilate);
        dilateImage = im2uint16(dilateImage);
        
        
        % apply MEA*************************        
        
        tic
        disp(' ---------------Time to execute for MEA* algorithm----------------');
        %disp('get cost marix')
        CostMatrixofImageModified = MEA( dilateImage, xs, ys,xd, yd);
        %disp('get path point')
        [pathPlot1, count] = wavefrontpath8ver2( CostMatrixofImageModified,  xs, ys,xd, yd);
        %disp('get infelcitonpoint')
        %inflection points of the modified wavefront path
         pathPlot1 = pathPlot1(end:-1:1,:);
        %[inflectionPointsmod] = findInflectionPoint( pathPlot1);
        disp('---------- Displaying Pruned Path V4-----------')
        [prunedPointsmod4,pruneSumtotal]= findprunedpathv4( pathPlot1, dilateImage);
        
        totalTime = toc
        disp('No of Turn of Proposed')
        [TunrnsSize] = findInflectionPoint( prunedPointsmod4);
        turns = size(TunrnsSize,1)-2
        %turns = size(prunedPointsmod4,1)-2
        
        
        [pruneSum] = pathCost(prunedPointsmod4)
        tempImage = imcomplement(finalImage);
        
        
        disp('Path Length of Proposed')
        disp(pruneSum)
        
        %      set (gca,'XDir','reverse');
        %         set( gcf, 'Color', [1 , 1 , 1]);
        %
        %xlswrite('\MEAPath.xls',prunedPointsmod4,'1','A1');
        figure('Name', 'Proposed Approach Path ', 'NumberTitle','off'), imagesc(rgbI);%tempImage);
        hold on
        disp('---------- Displaying Pruned-----------')
        %  plot( pathPlot1(:,2), pathPlot1(:,1),'LineWidth',2, 'Color','b');
%         scatter(inflectionPointsmod(:,2), inflectionPointsmod(:,1),'filled','bo');
%         scatter(prunedPointsmod4(:,2), prunedPointsmod4(:,1),'filled','yo');
        plot( prunedPointsmod4(:,2), prunedPointsmod4(:,1),'LineWidth',2, 'Color','b');
        scatter(P(2,2), P(2,1),'filled','ro');
        scatter(P(1,2), P(1,1),'filled','go');
        text(ys,xs+2,'S'); %,'FontSize',12,'FontWeight','bold');
        text(yd,xd+2,'G'); %,'FontSize',12,'FontWeight','bold');
        
        hold off
        lw = 1;      % LineWidth
        msz = 8;       % MarkerSize
        fsz = 6;      % Fontsize
        width = 3;     % Width in inches
        height = 3;    % Height in inches
        
        ylabel(' Y Position (m) ');
        xlabel({' X Position (m) '; strcat('Path Length=', num2str(round(pruneSum,4)), ' m : Turns =',num2str(turns,4)) ; strcat('Time =', num2str(round(totalTime ,4)), ' Secs : Total Processed cells=', num2str(count)) });
        set(gcf,'InvertHardcopy','on');
        set(gcf,'PaperUnits', 'inches');
        papersize = get(gcf, 'PaperSize');
        left = (papersize(1)- width)/2;
        bottom = (papersize(2)- (height))/2;
        myfiguresize = [left, bottom, width, height];
        set(gcf,'PaperPosition', myfiguresize);
        set(gca, 'FontSize', fsz); %<- Set properties 6
        print(strcat('MEA', mapName,'.jpg'),'-dtiff','-r300');
        disp('image saved')
        rgbI =  imread('InputImages\M5US.jpg');
        
        figure,imagesc(1:h, 1:w, rgbI); hold on;
        %colormap gray;
         % Visualize the desired path
         path = [prunedPointsmod4(:,2) prunedPointsmod4(:,1)]; 
                 plot(path(:,1), path(:,2),'k--d')  ; 
        scatter(ys, xs+2,'filled','ro');
        scatter(yd,xd+2,'filled','go');
        text(ys,xs+2,'S','FontSize',12,'FontWeight','bold');
        text(yd,xd+2,'G','FontSize',12,'FontWeight','bold');
        hold off;
        driveRobot(path);
        
    else
        disp (' Point lies on the OBSTACLE');
        
        
    end
end