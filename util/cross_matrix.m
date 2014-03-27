function omega = cross_matrix(axis, r)

omega=zeros(3,3);
n=abs(axis);
omega=[ 0    -r(3,n)  r(2,n);
      r(3,n)    0    -r(1,n);
     -r(2,n)  r(1,n)    0];
if axis < 0
    omega=-omega;
end

return
