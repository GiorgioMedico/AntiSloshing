%TRINTERP Interpolate homogeneous transformations
%
% T = TRINTERP(T0, T1, S) is a homogeneous transform (4x4) interpolated
% between T0 when S=0 and T1 when S=1.  T0 and T1 are both homogeneous
% transforms (4x4).  Rotation is interpolated using quaternion spherical
% linear interpolation (slerp).  If S (Nx1) then T (4x4xN) is a sequence of
% homogeneous transforms corresponding to the interpolation values in S.
%
% T = TRINTERP(T1, S) as above but interpolated between the identity matrix
% when S=0 to T1 when S=1.
%
% See also CTRAJ, QUATERNION.



% Copyright (C) 1993-2015, by Peter I. Corke
%
% This file is part of The Robotics Toolbox for MATLAB (RTB).
% 
% RTB is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% RTB is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
% 
% You should have received a copy of the GNU Leser General Public License
% along with RTB.  If not, see <http://www.gnu.org/licenses/>.
%
% http://www.petercorke.com

function T = trinterp_circular(A, B, C, D)

    if nargin == 4
        T0 = A; T1 = B; T2 = C; r = D;

%         if length(r) > 1
%             T = [];
%             for rr=r(:)'
%                 TT = trinterp(T0, T1, rr);
%                 T = cat(3, T, TT);
%             end
%             return;
%         end

        q0 = Quaternion(T0);
        q1 = Quaternion(T2);

        p0 = transl(T0);
        p1 = transl(T1);
        p2 = transl(T2);

        qr = q0.interp(q1, r);

        
        [center,rad,v1,v2] = circlefit3d(p0',p1',p2');
        pr = center' + v1'*rad*cos(r*pi)+v2'*rad*sin(r*pi);
%         
        
%     elseif nargin == 2
%     %	TR = TRINTERP(T, r)
%         T0 = A; r = B;
% 
%         if length(r) > 1
%             T = [];
%             for rr=r(:)'
%                 TT = trinterp(T0, rr);
%                 T = cat(3, T, TT);
%             end
%             return;
%         end
% 
%         q0 = Quaternion(T0);
%         p0 = transl(T0);
% 
%         qr = q0.scale(r);
%         pr = r*p0;
    else
        error('must be 4 arguments');
    end
    
    T = rt2tr(qr.R, pr);
        
