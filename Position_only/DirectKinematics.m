    function T0 = DirectKinematics(DH)

%Initialization
n       =       size(DH,1);     % Taking the number of joints
T       =       zeros(4,4,n);   %Homogeneous transformation between consecutive frames
T0      =       zeros(4,4,n);   %Homegeneous tranformation with respect to base frame

%%
%Calculating homogeneous transformation between consecutive frames
for i=1:n
    T(:,:,i) = Homogeneous(DH(i,:));
end

%%
%Calculating homegeneous tranformation with respect to base frame
T0(:,:,1) = T(:,:,1);

for i=2:n
    T0(:,:,i) = T0(:,:,i-1) * T(:,:,i);
end

end