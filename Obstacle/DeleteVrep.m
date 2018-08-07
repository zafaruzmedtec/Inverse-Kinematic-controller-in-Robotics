% Destructor
function DeleteVrep(clientID, vrep)

vrep.simxPauseSimulation(clientID,vrep.simx_opmode_oneshot_wait); % pause simulation
%   vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait); % stop simulation
vrep.simxFinish(clientID);  % close the line if still open
vrep.delete();              % call the destructor!
disp('simulation ended');

end