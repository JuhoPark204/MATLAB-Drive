function q = eul2quat(eul, sequence)
%EUL2QUAT  Convert Euler angles to quaternion (toolbox-free drop-in).
%   q = EUL2QUAT(EUL) uses the default 'ZYX' sequence.
%   q = EUL2QUAT(EUL, SEQUENCE) supports 3-letter axis sequences such as
%   'ZYX', 'ZYZ', 'XYZ', etc.
%
%   EUL is an N-by-3 matrix of Euler angles in radians, one set per row,
%   given in the order the axes appear in SEQUENCE (intrinsic rotations).
%   Q is an N-by-4 matrix of quaternions [w x y z], scalar part first.
%
%   This is a minimal replacement for the Robotics/Navigation/UAV Toolbox
%   eul2quat, implemented so the simulation runs without those toolboxes.

    if nargin < 2
        sequence = 'ZYX';
    end
    sequence = upper(sequence);

    % Build the composite quaternion as the product of three elementary
    % axis rotations, applied intrinsically in sequence order.
    n = size(eul, 1);
    q = zeros(n, 4);
    for k = 1:n
        q1 = axisQuat(sequence(1), eul(k,1));
        q2 = axisQuat(sequence(2), eul(k,2));
        q3 = axisQuat(sequence(3), eul(k,3));
        q(k,:) = quatMul(quatMul(q1, q2), q3);
    end
end

function q = axisQuat(axis, angle)
    h = angle / 2;
    c = cos(h);
    s = sin(h);
    switch axis
        case 'X'
            q = [c, s, 0, 0];
        case 'Y'
            q = [c, 0, s, 0];
        case 'Z'
            q = [c, 0, 0, s];
        otherwise
            error('eul2quat:badAxis', 'Unsupported axis ''%c''.', axis);
    end
end

function q = quatMul(a, b)
    % Hamilton product, [w x y z] convention.
    w1 = a(1); x1 = a(2); y1 = a(3); z1 = a(4);
    w2 = b(1); x2 = b(2); y2 = b(3); z2 = b(4);
    q = [ w1*w2 - x1*x2 - y1*y2 - z1*z2, ...
          w1*x2 + x1*w2 + y1*z2 - z1*y2, ...
          w1*y2 - x1*z2 + y1*w2 + z1*x2, ...
          w1*z2 + x1*y2 - y1*x2 + z1*w2 ];
end
