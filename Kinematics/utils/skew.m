function sk = skew(v)
% sk=skew(v)
%Build the antisymmetric matrix associated with vector v
% v is a 3x1 column vector or a 1x3 row vector with v1 v2 and v3 as
% scalar components.
%sk is a 3x3 antysimmetric matrix defined as
%       |0	   -v3     v2|
%sk=    |v3     0     -v1|   
%       |-v2    v1      0|
% The scalar product of v and a vector A is determined by
% sk*A = v cross product A
     
    sk=zeros(3);    %preallocate matrix sk as a 3x3 zeros
    
                    sk(1,2)=-v(3); sk(1,3)= v(2);
    sk(2,1)= v(3);                 sk(2,3)=-v(1);
    
    sk(3,1)=-v(2);  sk(3,2)= v(1); 
       
end
    