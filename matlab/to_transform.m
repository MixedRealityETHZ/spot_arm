% position should be vector with [x, y, z]
% quaternion should a vector with [w, x, y, z]
function T = to_transform(position, quaternion)
    R = quat2rotm(quaternion);
    T = [   R,    position';
         0, 0, 0,         1];
end