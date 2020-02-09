function [U, V] = fixSignsRotation(U,V)
    %Fix the signs
    Temp=abs(U);
    [junk,I]=max(Temp,[],1);
    %[sign(U(I(1),1)),sign(U(I(2),2)),sign(U(I(3),3))]
    Signs=diag([sign(U(I(1),1)),sign(U(I(2),2)),sign(U(I(3),3))]);
    U=U*Signs;
    V(:,1:3)=V(:,1:3)*Signs;
end