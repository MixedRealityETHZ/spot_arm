% Quaternions are in form [w, x, y, z]
function q_interp = interp_quaternion(t1, t2, q1, q2, t_query)
    interp_coeff = (t_query - t1) / (t2 - t1);
    q1 = q1 / norm(q1);
    q2 = q2 / norm(q2);
    if interp_coeff < 0 || interp_coeff > 1
        error("interp coefficient wasn't between 0 and 1");
    end
    q_interp = quatinterp(q1, q2, interp_coeff, "slerp");
end