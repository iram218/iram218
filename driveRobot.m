function [ output_args ] = driveRobot( path )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
% Author : Iram Noreen                 
%path
               % plot( prunedPointsmod4(:,2), prunedPointsmod4(:,1),'LineWidth',2, 'Color','r','LineStyle','--');
        %         xlim([0 13])
        %         ylim([0 13])
        %
                % Set the current location and the goal location of the robot as defined by the path
%                 nx(203) = zeros();
%                 ny(203) = zeros();
%                 t2=1;
%                 for i = 1 : size(path,1)-1
%                     x1 = path(i,1);
%                     x2 = path(i+1, 1);
%                     y1 = path(i,2);
%                     y2 = path(i+1, 2);            
%                
%                 for t1  =  0:0.01:1
%                 % have removed the ceil and used for loop
%                     nx(t2) = (t1*x1 + (1-t1)*x2);
%                     ny(t2) = (t1*y1 + (1-t1)*y2); 
%                      t2 = t2 +1;
%                 end
%                 end
%                 disp('t2  = ');
%                t2
                              
                rosinit;
                robotCurrentLocation = path(1,:);
                robotGoal = path(end,:);
                initialOrientation = 0;
                % Define the current pose for the robot [x y theta]
                robotCurrentPose = [robotCurrentLocation initialOrientation];
        
                robot = ExampleHelperDifferentialDriveRobot(robotCurrentPose);
        
                controller = robotics.PurePursuit
        
                controller.Waypoints = path;
                controller.DesiredLinearVelocity = 0.1; %1.5; % 0.3
                controller.MaxAngularVelocity = 1;%2;
                controller.LookaheadDistance = 1;%0.9; % 0.5
                goalRadius = 0.5; % 0.2
                distanceToGoal = norm(robotCurrentLocation - robotGoal);
        
                % The |<docid:robotics_ref.buopp2z-1 step>| function computes control commands for the robot.
                % Drive the robot using these control commands until it reaches within the
                % goal radius. If you are using an external simulator or a physical robot,
                % then the controller outputs should be applied to the robot and a localization
                % system may be required to update the pose of the robot.
                tic
              %  i = 0;
                while( distanceToGoal > goalRadius )
             %       i = i + 1;
                    % Compute the controller outputs, i.e., the inputs to the robot
                    [v, omega] = step(controller, robot.CurrentPose);
                    % Simulate the robot using the controller outputs.
                    drive(robot, v, omega)
                    % Extract current location information ([X,Y]) from the current pose of the robot
                    robotCurrentLocation = robot.CurrentPose(1:2);               
                    % Re-compute the distance to the goal
                    distanceToGoal = norm(robotCurrentLocation - robotGoal);
                end
                toc
            rosshutdown;
            %i

             % plot(nx(:), ny(:),'m--d')  ; 
end

