function R = eul2rotm(eul, sequence)
%EUL2ROTM  Convert Euler angles to rotation matrix (toolbox-free drop-in).
%   R = EUL2ROTM(EUL) uses the default 'ZYX' sequence.
%   R = EUL2ROTM(EUL, SEQUENCE) supports 3-letter axis sequences.
%
%   EUL is an N-by-3 matrix of Euler angles in radians, one set per row,
%   in the order the axes appear in SEQUENCE (intrinsic rotations).
%   R is a 3-by-3-by-N array of rotation matrices.
%
%   Minimal replacement for the Robotics/Navigation/UAV Toolbox eul2rotm.

    if nargin < 2
        sequence = 'ZYX';
    end
    sequence = upper(sequence);

    n = size(eul, 1);
    R = zeros(3, 3, n);
    for k = 1:n
        Rk = axisRot(sequence(1), eul(k,1)) * ...
             axisRot(sequence(2), eul(k,2)) * ...
             axisRot(sequence(3), eul(k,3));
        R(:,:,k) = Rk;
    end
end

function M = axisRot(axis, a)
    c = cos(a);
    s = sin(a);
    switch axis
        case 'X'
            M = [1 0 0; 0 c -s; 0 s c];
        case 'Y'
            M = [c 0 s; 0 1 0; -s 0 c];
        case 'Z'
            M = [c -s 0; s c 0; 0 0 1];
        otherwise
            error('eul2rotm:badAxis', 'Unsupported axis ''%c''.', axis);
    end
end
