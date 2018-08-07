function [t, q, q_act] = main

close all

kuka_init

porta   = 19997;        % default V-REP port
n       = 7;            % number of joints
tf      = 4;            % final time
Ts      = 0.01;         % sampling time
t       = 0:Ts:tf;      % time vector
N       = length(t);    % number of points of the simulation

T               = DirectKinematics(DH);
ee_pos_initial  = [-0.55, -0.32,  0.31]';
ee_pos_final    = [-0.55,  0.32,  0.25]';
ee_quat_final   = Rot2Quat(T(1:3,1:3,n));
ee_vel_cruize   = [1.5 ,  1.5 ,  1.5 ]' .* abs(ee_pos_final - ee_pos_initial) / tf;

q     = zeros(n,N);           % q(:,i) collects the joint position for t(i)
qvrep = zeros(n,N);
q_dot = zeros(n,N);           % dq(:,i) collects the joint velocity for t(i)

pos_trpz      = zeros(3,N);
vel_trpz      = zeros(3,N);
acc_trpz      = zeros(3,N);
final_pos_err = zeros(3,N);
pos_err       = zeros(3,N);
quat_err      = zeros(3,N);

q(:,1)     = DH(:,4);
qvrep(:,1) = q(:,1);
qvrep(2,1) = -qvrep(2,1);
qvrep(6,1) = -qvrep(6,1);

%% Start of simulation
clc
fprintf('----------------------');
fprintf('\n Simulation started ');
fprintf('\n Trying to connect...\n');
[clientID, vrep ] = StartVrep(porta);

[returnCode]= vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot);
pause(2)

handle_joint = my_get_handle_Joint(vrep,clientID);      % handle to the joints

my_set_joint_target_position(vrep, clientID, handle_joint, qvrep(:,1)); % first move to q0
q_act(:,1) = my_get_joint_target_position(clientID,vrep,handle_joint,n);% get the actual joints angles from v-rep
pause(2)

%% Main simulation loop
for i=1:N
    DH(:,4) = q(:,i);
    [pos_trpz(:,i), vel_trpz(:,i), acc_trpz(:,i)] = Trapezoidal(ee_pos_initial, ee_pos_final, ee_vel_cruize, tf, t(i));
    [q_dot(:,i), final_pos_err(:,i), pos_err(:,i), quat_err(:,i)] = InverseKinematics(ee_pos_final, pos_trpz(:,i), ee_quat_final,  vel_trpz(:,i), DH);
    
        %Avoid obstacle
    dwq = [0 0 0 0 0 0 0];
    d = 0.01;
    e = diag([d*[1 1 1], d*[1 1 1 1]]);
    k = 0.5;
    for j = 1:7
        dwq(:,j) = (function_w(q(:,i)+e(:,j))-function_w(q(:,i)))/d;
    end
    
    q_a = k * dwq';
    I = eye(7,7);
    J = Jacobian(DH);
    q_dot(:,i) = q_dot(:,i)+(I-pinv(J)*J)*q_a;
    
    if i<N
        q(:,i+1) = q(:,i) + Ts * q_dot(:,i);
        J = Jacobian(DH);
        vel_trpz(:,i) = J(1:3,:)*q_dot(:,i);
    end
    
    qvrep(:,i) = q(:,i);
    qvrep(2,i) = -qvrep(2,i);
    qvrep(6,i) = -qvrep(6,i);
    
    my_set_joint_target_position(vrep, clientID, handle_joint, qvrep(:,i));
    q_act(:,i) = my_get_joint_target_position(clientID,vrep,handle_joint,n);% get the actual joints angles from v-rep
    
end
DeleteVrep(clientID, vrep);

%% Plot graphs

figure
subplot(211)
plot(t,final_pos_err)
title('Position Error')
xlabel('Time(s)')
ylabel('Error(m)')
legend('x', 'y', 'z')
grid on
subplot(212)
plot(t,quat_err)
title('Orientation Error')
xlabel('Time(s)')
ylabel('Error(rad)')
legend('x', 'y', 'z')
grid on


figure
subplot(211)
plot(t,pos_trpz)
title('End Effector Position')
xlabel('Time(s)')
ylabel('Position(m)')
legend('x', 'y', 'z')
grid on
subplot(212)
plot(t,vel_trpz)
title('End Effector Velocity')
xlabel('Time(s)')
ylabel('Linear velocity(m/s)')
legend('x', 'y', 'z')
grid on


figure
subplot(211)
plot(t,q)
grid on
title('Joint Positions')
xlabel('Time(s)')
ylabel('Position(rad)')
legend('q1', 'q2', 'q3', 'q4', 'q5', 'q6', 'q7')
subplot(212)
plot(t,q_dot)
grid on
title('Joint Velocities')
xlabel('Time(s)')
ylabel('Angular velocity(rad/s)')
legend('dq1', 'dq2', 'dq3', 'dq4', 'dq5', 'dq6', 'dq7')


figure
hold on
DH(:,4) = q(:,1);
DrawRobot(DH);
DH(:,4) = q(:,N);
DrawRobot(DH);

end

%% VREP related functions
% Constructor
function [clientID, vrep ] = StartVrep(porta)

vrep = remApi('remoteApi');   % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1);        % just in case, close all opened connections
clientID = vrep.simxStart('127.0.0.1',porta,true,true,5000,5);% start the simulation

if (clientID>-1)
    disp('remote API server connected successfully');
else
    disp('failed connecting to remote API server');
    DeleteVrep(clientID, vrep); %call the destructor!
end
% to change the simulation step time use this command below, a custom dt in v-rep must be selected,
% and run matlab before v-rep otherwise it will not be changed
% vrep.simxSetFloatingParameter(clientID, vrep.sim_floatparam_simulation_time_step, 0.002, vrep.simx_opmode_oneshot_wait);

end

% Destructor
function DeleteVrep(clientID, vrep)

vrep.simxPauseSimulation(clientID,vrep.simx_opmode_oneshot_wait); % pause simulation
%   vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait); % stop simulation
vrep.simxFinish(clientID);  % close the line if still open
vrep.delete();              % call the destructor!
disp('simulation ended');

end

function my_set_joint_target_position(vrep, clientID, handle_joint, q)

[m,n] = size(q);
for i=1:n
    for j=1:m
        err = vrep.simxSetJointTargetPosition(clientID,handle_joint(j),q(j,i),vrep.simx_opmode_oneshot);
        if (err ~= vrep.simx_error_noerror)
            fprintf('failed to send joint angle q %d \n',j);
        end
    end
end

end

function handle_joint = my_get_handle_Joint(vrep,clientID)

[~,handle_joint(1)] = vrep.simxGetObjectHandle(clientID,'LBR_iiwa_7_R800_joint1',vrep.simx_opmode_oneshot_wait);
[~,handle_joint(2)] = vrep.simxGetObjectHandle(clientID,'LBR_iiwa_7_R800_joint2',vrep.simx_opmode_oneshot_wait);
[~,handle_joint(3)] = vrep.simxGetObjectHandle(clientID,'LBR_iiwa_7_R800_joint3',vrep.simx_opmode_oneshot_wait);
[~,handle_joint(4)] = vrep.simxGetObjectHandle(clientID,'LBR_iiwa_7_R800_joint4',vrep.simx_opmode_oneshot_wait);
[~,handle_joint(5)] = vrep.simxGetObjectHandle(clientID,'LBR_iiwa_7_R800_joint5',vrep.simx_opmode_oneshot_wait);
[~,handle_joint(6)] = vrep.simxGetObjectHandle(clientID,'LBR_iiwa_7_R800_joint6',vrep.simx_opmode_oneshot_wait);
[~,handle_joint(7)] = vrep.simxGetObjectHandle(clientID,'LBR_iiwa_7_R800_joint7',vrep.simx_opmode_oneshot_wait);

end

function my_set_joint_signal_position(vrep, clientID, q)

[~,n] = size(q);

for i=1:n
    joints_positions = vrep.simxPackFloats(q(:,i)');
    [err]=vrep.simxSetStringSignal(clientID,'jointsAngles',joints_positions,vrep.simx_opmode_oneshot_wait);
    
    if (err~=vrep.simx_return_ok)
        fprintf('failed to send the string signal of iteration %d \n',i);
    end
end
pause(8);% wait till the script receives all data, increase it if dt is too small or tf is too high

end

function angle = my_get_joint_target_position(clientID,vrep,handle_joint,n)

for j=1:n
    vrep.simxGetJointPosition(clientID,handle_joint(j),vrep.simx_opmode_streaming);
end

pause(0.05);

for j=1:n
    [err(j),angle(j)]=vrep.simxGetJointPosition(clientID,handle_joint(j),vrep.simx_opmode_buffer);
end

if (err(j)~=vrep.simx_return_ok)
    fprintf(' failed to get position of joint %d \n',j);
end

end

