%% Path
path = [2,1;1.25,1.75; 5.25,8.25;7.25,8.75;11.75,10.75]; % goal =[(1,2); (3,4)]
robotInitialLocation = path(1,:);
robotGoal = path(end,:);
initialOrientation = 0;
robotCurrentPose = [robotInitialLocation initialOrientation]';
robot = differentialDriveKinematics("TrackWidth", 1, "VehicleInputs", "VehicleSpeedHeadingRate"); % find out 
figure
plot(path(:,1), path(:,2),'k--d')
xlim([0 13])
ylim([0 13])
hold on
%% colone another robot
robot2 = differentialDriveKinematics("TrackWidth", 1, "VehicleInputs", "VehicleSpeedHeadingRate");
path2 = [3,1;2.25,1.75; 6.25,8.25;8.25,8.75;12.75,10.75];
robot2InitialLocation = path2(1,:);
robot2Goal = path2(end,:);
initialOrientation2 = 0;
robot2CurrentPose = [robot2InitialLocation initialOrientation2]';
plot(path2(:,1), path2(:,2),'k--d')

%% controller
controller = controllerPurePursuit;
controller.Waypoints = path;
controller.DesiredLinearVelocity = 0.3;
controller.MaxAngularVelocity = 2;
controller.LookaheadDistance = 0.3;
controller2 = controllerPurePursuit;
controller2.Waypoints = path2;
controller2.DesiredLinearVelocity = 1;
controller2.MaxAngularVelocity = 1.7;
controller2.LookaheadDistance = 0.3;
goalRadius = 0.3;
goalRadius2 = 1;
distanceToGoal = norm(robotInitialLocation - robotGoal);
distanceToGoal2 = norm(robot2InitialLocation - robot2Goal);
% Initialize the simulation loop
sampleTime = 0.1; % what is sample time and vizRate?
vizRate = rateControl(1/sampleTime);  
% Initialize the figure
%figure
% Determine vehicle frame size to most closely represent vehicle with plotTransforms
frameSize = robot.TrackWidth/0.8;

%% actuator
while( distanceToGoal2 > goalRadius2 && distanceToGoal > goalRadius) % 1. displaying two robots in the same position 2. the second robot does not on screen
    % Compute the controller outputs, i.e., the inputs to the robot
    [v, omega] = controller(robotCurrentPose);
    [v2, omega2] = controller2(robot2CurrentPose); 
    % Get the robot's velocity using controller inputs
    vel = derivative(robot, robotCurrentPose, [v omega]);
    vel2 = derivative(robot2, robot2CurrentPose, [v2 omega2]);
    
    % Update the current pose
    robotCurrentPose = robotCurrentPose + vel*sampleTime; 
    robot2CurrentPose = robot2CurrentPose + vel2*sampleTime; 
    
    % Re-compute the distance to the goal
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal(:));    
    distanceToGoal2 = norm(robot2CurrentPose(1:2) - robot2Goal(:));
    
    % Update the plot
    hold off
    
    % Plot path each instance so that it stays persistent while robot mesh
    % moves
    plot(path2(:,1), path2(:,2),"k--d")
    hold on
    plot(path(:,1), path(:,2),"k--d")
    hold all
    % Plot the path of the robot as a set of transforms
    plotTrVec = [robotCurrentPose(1:2); 0];
    hold on
    plotTrVec2 = [robot2CurrentPose(1:2); 0];
    plotRot = axang2quat([0 0 1 robotCurrentPose(3)]);
    plotRot2 = axang2quat([0 0 1 robot2CurrentPose(3)]);
    plotTransforms(plotTrVec', plotRot, "MeshFilePath", "groundvehicle.stl", "Parent", gca, "View","2D", "FrameSize", frameSize);
    plotTransforms(plotTrVec2', plotRot2, "MeshFilePath", "groundvehicle.stl", "Parent", gca, "View","2D", "FrameSize", frameSize);
    light;
    xlim([0 15])
    ylim([0 15])
    waitfor(vizRate);
end
% 
% while( distanceToGoal2 > goalRadius2 || distanceToGoal > goalRadius)
%     if (distanceToGoal2 > goalRadius2)
%         robot2CurrentPose = robot2CurrentPose + vel2*sampleTime; 
%         distanceToGoal2 = norm(robot2CurrentPose(1:2) - robot2Goal(:));
%         
%         
% 
%     end
% end
% 


























