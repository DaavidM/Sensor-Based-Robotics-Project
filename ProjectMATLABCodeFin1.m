
vrep=remApi('remoteApi');

vrep.simxFinish(-1);
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

if (clientID>-1)
    disp('Connected')
   
    double angle_1; 
    double angle_2;
    
    angle_1 = (-.7)*pi;
    angle_2 = (-.49)*pi;
    
    % Naming Joint Variables
    
    [returnCode,joint_1]=vrep.simxGetObjectHandle(clientID,'Revolute_joint',vrep.simx_opmode_blocking);
    [returnCode,joint_2]=vrep.simxGetObjectHandle(clientID,'Revolute_joint0',vrep.simx_opmode_blocking);
    [returnCode,link_1]=vrep.simxGetObjectHandle(clientID,'Cuboid0',vrep.simx_opmode_blocking);
    [returnCode,link_2]=vrep.simxGetObjectHandle(clientID,'Cuboid1',vrep.simx_opmode_blocking);
    
    % Naming Motor Variables
    
    [returnCode,left_Motor]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',vrep.simx_opmode_blocking)
    [returnCode,right_Motor]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor',vrep.simx_opmode_blocking)
    
    % Motor Pathway
    
    [returnCode] = vrep.simxSetJointTargetVelocity(clientID, left_Motor, 2 ,vrep.simx_opmode_blocking) % Initial turn
    
    pause(1.1);
    
    [returnCode] = vrep.simxSetJointTargetVelocity(clientID, left_Motor,0,vrep.simx_opmode_blocking)
    [returnCode] = vrep.simxSetJointTargetVelocity(clientID, right_Motor, 2, vrep.simx_opmode_blocking)
    [returnCode] = vrep.simxSetJointTargetVelocity(clientID, left_Motor, 2.6 ,vrep.simx_opmode_blocking)
    
    pause(1.2);
    
    [returnCode] = vrep.simxSetJointTargetVelocity(clientID, right_Motor,0,vrep.simx_opmode_blocking)
    [returnCode] = vrep.simxSetJointTargetVelocity(clientID, left_Motor,0,vrep.simx_opmode_blocking)
    [returnCode] = vrep.simxSetJointTargetVelocity(clientID, right_Motor,5,vrep.simx_opmode_blocking)
    [returnCode] = vrep.simxSetJointTargetVelocity(clientID, left_Motor,2.3,vrep.simx_opmode_blocking)
   
    
    pause(1);
    
    %Start: Avoid Cuboid #1 with robot manipulator.
    [returnCode] = vrep.simxSetJointTargetPosition(clientID,joint_1,angle_1,vrep.simx_opmode_blocking);
    % End
    [returnCode] = vrep.simxSetJointTargetVelocity(clientID, right_Motor,3,vrep.simx_opmode_blocking);
    [returnCode] = vrep.simxSetJointTargetVelocity(clientID, left_Motor,2.4,vrep.simx_opmode_blocking);
    
    pause(1);
    
    [returnCode] = vrep.simxSetJointTargetVelocity(clientID, right_Motor,1,vrep.simx_opmode_blocking);
    [returnCode] = vrep.simxSetJointTargetVelocity(clientID, left_Motor,2,vrep.simx_opmode_blocking);
    
    pause(0.5)
    
    [returnCode] = vrep.simxSetJointTargetVelocity(clientID, left_Motor,1.2,vrep.simx_opmode_blocking);
    [returnCode] = vrep.simxSetJointTargetVelocity(clientID, right_Motor,2,vrep.simx_opmode_blocking);
    
    
    pause(1)
    
    % Start: Avoid Obstacle/Cuboid #2 before stopping.
    [returnCode] = vrep.simxSetJointTargetPosition(clientID,joint_1,angle_2,vrep.simx_opmode_blocking);
    % End
    
    [returnCode] = vrep.simxSetJointTargetVelocity(clientID, left_Motor,5,vrep.simx_opmode_blocking);
    
    [returnCode] = vrep.simxSetJointTargetVelocity(clientID, right_Motor,5,vrep.simx_opmode_blocking);
    
    pause(1)
    
    [returnCode] = vrep.simxSetJointTargetVelocity(clientID, left_Motor,0,vrep.simx_opmode_blocking);
    
    [returnCode] = vrep.simxSetJointTargetVelocity(clientID, right_Motor,0,vrep.simx_opmode_blocking);
    
    %Motor Pathway Done
    
    %Position Reporting Initial
    [returnCode,position_1]=vrep.simxGetObjectPosition(clientID, link_1, -1, vrep.simx_opmode_streaming);
    [returnCode,position_2]=vrep.simxGetObjectPosition(clientID, link_2, -1, vrep.simx_opmode_streaming);
    [returnCode,joint_Pos1]=vrep.simxGetObjectPosition(clientID, joint_1, -1, vrep.simx_opmode_streaming);
    [returnCode,joint_Pos2]=vrep.simxGetObjectPosition(clientID, joint_2, -1, vrep.simx_opmode_streaming);
    
    
    for i=1:15000
    [returnCode,position_1]=vrep.simxGetObjectPosition(clientID, link_1, -1, vrep.simx_opmode_buffer);
    [returnCode,position_2]=vrep.simxGetObjectPosition(clientID, link_2, -1, vrep.simx_opmode_buffer);
    [returnCode,joint_Pos1]=vrep.simxGetObjectPosition(clientID, joint_1, -1, vrep.simx_opmode_buffer);
    [returnCode,joint_Pos2]=vrep.simxGetObjectPosition(clientID, joint_2, -1, vrep.simx_opmode_buffer);
    end
    
    disp('First Joint Position')
    disp(joint_Pos1)
    disp('Second Joint Position')
    disp(joint_Pos2)
    disp('First Link Position: ')
    disp(position_1)
    disp('Second Link Position: ')
    disp(position_2)
  
end

vrep.delete()
