function sk=skew(v)
% sk=skew(v)
%Build the antisymmetric matrix associated with vector v
% v is a 3x1 column vector or a 1x3 row vector with v1 v2 and v3 as
% scalar components.
%sk is a 3x3 antysimmetric matrix defined as
%       |0	   -v3    v2|
%sk=    |v3     0     -v1|   
%       |-v2    v1     0|
% The scalar product of v and a vector A is determined by
% sk*A = v cross product A
     
    sk=zeros(3);    %preallocate matrix sk as a 3x3 zeros
    sk(1,2)=-v(3);
    sk(1,3)= v(2);
    sk(2,1)= v(3);
    sk(2,3)=-v(1);
    sk(3,1)=-v(2);
    sk(3,2)= v(1);
    
    
% DA FARE PIù AVANTI------------------------------------------------------------            
% % -exercise--> if dimension of v is not equal to 3, print an error message
% 
% %You will use the command 'length'. First read help for both 'length' 
% and 'size' and see the differences.'size' returns a vector with two 
% elements(number of row and number of columns of the variable) . 
%'length' take the biggest of the two, thus returning a scalar. 
% length(v) is the same as computing max(size(v)).
% 
%This is the code for the error message. If you are interested check 'disp'
%and 'return'. You can paste the code  before computing sk (line 18)
%
%if length(v)~=3
%  disp('Vector must have dimension 3 in order to define the correspondent screw matrix')
%  disp(' ')
%  return
%end  
    

