function quat_e = QuatError (quat_d, quat_c)

quat_e = quat_c(4)*quat_d(1:3, 1)   -   quat_d(4)*quat_c(1:3, 1)   -   cross(quat_d(1:3, 1), quat_c(1:3, 1));

end