function quat = Rot2Quat(rot)

quat = zeros(4,1);

quat(1) = (1/2) * sign(rot(3,2) - rot(2,3)) * sqrt(rot(1,1) - rot(2,2) - rot(3,3));
quat(2) = (1/2) * sign(rot(1,3) - rot(3,1)) * sqrt(rot(2,2) - rot(1,1) - rot(3,3));
quat(3) = (1/2) * sign(rot(2,1) - rot(1,2)) * sqrt(rot(3,3) - rot(1,1) - rot(2,2));
quat(4) = (1/2) * sqrt(rot(1,1) + rot(2,2) + rot(3,3) + 1);

quat = real(quat);
quat = quat/norm(quat);

end