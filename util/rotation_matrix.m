function r = rotation_matrix(axis, theta)

r= eye(3);
if axis == 1
   r(2,2)=cos(theta); r(3,2)=sin(theta);
   r(2,3)=-r(3,2);    r(3,3)=r(2,2);
elseif axis == -1
   r(2,2)=cos(theta); r(2,3)=sin(theta);
   r(3,2)=-r(2,3);    r(3,3)=r(2,2);
elseif axis == 2
   r(1,1)=cos(theta); r(1,3)=sin(theta);
   r(3,1)=-r(1,3);    r(3,3)=r(1,1);
elseif axis== -2
   r(1,1)=cos(theta); r(3,1)=sin(theta);
   r(1,3)=-r(3,1);    r(3,3)=r(1,1);
elseif axis == 3
   r(1,1)=cos(theta); r(2,1)=sin(theta);
   r(1,2)=-r(2,1);    r(2,2)=r(1,1);
elseif axis == -3
   r(1,1)=cos(theta); r(1,2)=sin(theta);
   r(2,1)=-r(1,2);    r(2,2)=r(1,1);
end

return