function q = rotm2quat(R)
%ROTM2QUAT  Convert rotation matrix to quaternion (toolbox-free drop-in).
%   q = ROTM2QUAT(R) where R is 3-by-3-by-N returns an N-by-4 matrix of
%   quaternions [w x y z], scalar part first.
%
%   Minimal replacement for the Robotics/Navigation/UAV Toolbox rotm2quat.

    n = size(R, 3);
    q = zeros(n, 4);
    for k = 1:n
        M = R(:,:,k);
        tr = M(1,1) + M(2,2) + M(3,3);
        if tr > 0
            s = sqrt(tr + 1.0) * 2;      % s = 4*w
            w = 0.25 * s;
            x = (M(3,2) - M(2,3)) / s;
            y = (M(1,3) - M(3,1)) / s;
            z = (M(2,1) - M(1,2)) / s;
        elseif (M(1,1) > M(2,2)) && (M(1,1) > M(3,3))
            s = sqrt(1.0 + M(1,1) - M(2,2) - M(3,3)) * 2;  % s = 4*x
            w = (M(3,2) - M(2,3)) / s;
            x = 0.25 * s;
            y = (M(1,2) + M(2,1)) / s;
            z = (M(1,3) + M(3,1)) / s;
        elseif M(2,2) > M(3,3)
            s = sqrt(1.0 + M(2,2) - M(1,1) - M(3,3)) * 2;  % s = 4*y
            w = (M(1,3) - M(3,1)) / s;
            x = (M(1,2) + M(2,1)) / s;
            y = 0.25 * s;
            z = (M(2,3) + M(3,2)) / s;
        else
            s = sqrt(1.0 + M(3,3) - M(1,1) - M(2,2)) * 2;  % s = 4*z
            w = (M(2,1) - M(1,2)) / s;
            x = (M(1,3) + M(3,1)) / s;
            y = (M(2,3) + M(3,2)) / s;
            z = 0.25 * s;
        end
        qk = [w, x, y, z];
        if qk(1) < 0          % canonical form: non-negative scalar part
            qk = -qk;
        end
        q(k,:) = qk / norm(qk);
    end
end
