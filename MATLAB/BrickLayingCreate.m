% MATLAB code for doing the driving of the Create robot base to
% perform the brick-laying scenario.  It requires user-interaction
% in the form of entering carriage-returns when each phase of motion
% should be initiated. Note that the port number will probably need
% to be adjusted for your computer.
%
% This code relies upon the "Matlab Toolbox for the iRobot Create2 -
% MTIC2", available here:
%   http://www.usna.edu/Users/weapsys/esposito/roomba-matlab.php

function BrickLayingCreate()
    pause('on');
    portNum = 39; % adjust for your computer
    serPort = RoombaInit(portNum);
    %PowerOffRoomba(serPort);
    baseDistance = 0.35;
    spaceBetweenBricks = 0.05;
    distances = [ baseDistance, ...
        baseDistance + spaceBetweenBricks, ...
        baseDistance + spaceBetweenBricks, ...
        baseDistance + 2 * spaceBetweenBricks, ... 
        baseDistance + 2 * spaceBetweenBricks, ...
        baseDistance ]; ...   
    [~, numLegs] = size(distances);
 
    for leg = 0 : (numLegs/2 - 1)
        pause(); % wait for input
        % PICK BRICK UP HERE

        % do the turn & drive to brick-laying area
        angleInRadiansInitialRead = readAngleSensor(serPort);
        rotateRoomba180(serPort, angleInRadiansInitialRead, true);
        distanceInMetersInitalRead = readDistanceSensor(serPort);
        driveRoombaDist(serPort, distanceInMetersInitalRead, ...
            distances(1,2*leg+1));
        pause(); % wait for input
        % PUT BRICK DOWN HERE
    
        % do the turn & drive back to brick pick-up area
        angleInRadiansInitialRead = readAngleSensor(serPort);
        rotateRoomba180(serPort, angleInRadiansInitialRead, false);
        distanceInMetersInitalRead = readDistanceSensor(serPort);
        driveRoombaDist(serPort, distanceInMetersInitalRead, distances(1,2*leg+2));
    end
end

function driveRoombaDist(serPort, distanceInMetersInitalRead, distGoal)
    distTraveled = 0.0;
    epsilon = 0.005;
    loopcount = 0;
    while (distTraveled < distGoal - epsilon)
        loopcount = loopcount + 1;
        SetWheelVelRoomba(serPort, 0.05, 0.05);
        pause(0.1)
        distInMetersThisRead = DistanceSensorRoomba(serPort);
        distTraveled = distTraveled + distInMetersThisRead;
        diffDist = distTraveled - distanceInMetersInitalRead;
        
        testVal = distGoal - diffDist;
        if (testVal <= epsilon)
            fprintf('Terminating Loop: %d, Goal: %f:\n', ...
                    loopcount, distGoal);
            fprintf('  Init Dist: %f.\n', distanceInMetersInitalRead);
            fprintf('  Delta Dist: %f.\n', distInMetersThisRead);
            fprintf('  Total Dist: %f.\n', distTraveled);
            fprintf('  Meters Traveled: %f.\n', diffDist);
            break;
        end
        if (mod(loopcount, 10) == 0)
            fprintf('Loop: %d:\n', loopcount);
            fprintf('  Init Dist: %f.\n', distanceInMetersInitalRead);
            fprintf('  Delta Dist: %f.\n', distInMetersThisRead);
            fprintf('  Total Dist: %f.\n', distTraveled);
            fprintf('  Meters Traveled: %f.\n', diffDist);
        end
    end
    SetWheelVelRoomba(serPort, 0.00, 0.00);
end

function initDist = readDistanceSensor(serPort)
    DistanceSensorRoomba(serPort); % read once to reset to zero
    initDist = DistanceSensorRoomba(serPort);
end

function initAngleInRadians = readAngleSensor(serPort)
    AngleSensorRoomba(serPort); % read once to reset to zero
    initAngleInRadians = AngleSensorRoomba(serPort)
end

function rotateRoomba180(serPort, angleInRadiansInitialRead, positive)
    epsilon = 0.005;
    loopcount = 0;
    totalRotation = 0;
    if (positive)
        goalRaw = pi;
        rotVel = 0.1;
    else 
        goalRaw = -pi;
        rotVel = -0.1;
    end
    fudgeFactor = 1.14;
    goal = goalRaw * fudgeFactor;
    while true
        loopcount = loopcount + 1;
        SetFwdVelAngVelRoomba(serPort, 0.0, rotVel);
        pause(0.1);
        angleInRadiansThisRead = AngleSensorRoomba(serPort);
        totalRotation = totalRotation + angleInRadiansThisRead;
        diffAngle = totalRotation - angleInRadiansInitialRead;

        testVal = goal - diffAngle;
        if ((positive      && (testVal <= epsilon)) || ...
            (not(positive) && (testVal >= epsilon)))
            fprintf('Terminating Loop: %d, Goal: %f:\n', ...
                    loopcount, goal);
            fprintf('  Init Angle: %f.\n', angleInRadiansInitialRead);
            fprintf('  Delta Angle: %f.\n', angleInRadiansThisRead);
            fprintf('  Total Angle: %f.\n', totalRotation);
            fprintf('  Radians Rotated: %f.\n', diffAngle);
            break;
        end
        if (mod(loopcount, 10) == 0)
            fprintf('Loop: %d:\n', loopcount);
            fprintf('  Init Angle: %f.\n', angleInRadiansInitialRead);
            fprintf('  Delta Angle: %f.\n', angleInRadiansThisRead);
            fprintf('  Total Angle: %f.\n', totalRotation);
            fprintf('  Radians Rotated: %f.\n', diffAngle);
        end
    end
    SetFwdVelAngVelRoomba(serPort, 0.0, 0.0);
end
