function testData = funcCalculateuandR(testData)
testData.u10=[];
% Get xi and x0, ui and u0, quaternion of tip
x0=testData.tip_exp(:,2);
z0=testData.tip_exp(:,4);

xi=testData.tip2_exp(:,2);
zi=testData.tip2_exp(:,4);
u=2*(xi-x0)./((xi-x0).^2 + (zi-z0).^2);
testData.u10=u;
%% https://github.com/mkilling/PyNatNet/blob/master/libs/NatNetSDK/Samples/SampleClient3D/NATUtils.cpp
q=testData.tip2_RQ(:,2:4);
q(:,4)=testData.tip2_RQ(:,1);
testData.eul=[];
for i =1:length(q)
    testData.eul(i,:)=quat2eul(q(i,:),"XYZ");
end
end