function [dq, final_pos_err, pos_err, quat_err] = InverseKinematics(final_pos, pos_desired, quat_desired, vel_desired, DH)

algorithm = 'inverse';

% Current position and orientation
n    = size(DH,1);
T    = DirectKinematics(DH);
pos  = T(1:3,4,n);
quat = Rot2Quat(T(1:3,1:3,n));

% Initialize K
if strcmp(algorithm,'transpose')
    K = diag([30*[1 1 1] , 30*[1 1 1]]);
else
    K = diag([3*[1 1 1] , 3*[1 1 1]]);
    
end

% Calculate error
final_pos_err   = final_pos - pos;
pos_err         = pos_desired - pos;
quat_err = QuatError(quat_desired,quat);
error    = [pos_err; quat_err]; %

% Inverse kinematics algorithm
J = Jacobian(DH);
if strcmp(algorithm,'inverse')
    dq = pinv(J) * ( [vel_desired;0;0;0] + K*error );
else
    dq = J' * ( [vel_desired;0;0;0] + K*error );
end

end
`   
