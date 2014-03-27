function a = rotation_axis(axis, r)

a=zeros(3,1);
n=abs(axis);
a=[ r(1,n); r(2,n); r(3,n) ];
if axis < 0
    a=-a;
end

return 