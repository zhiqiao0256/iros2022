dhparams = [0   	-pi/2	0   	0; % a alpha d theta 
            0.      pi/2    0       0;
            0.      0		0       0;
            0   	-pi/2	0       0;
            0       pi/2	0   	0;];
robot = rigidBodyTree;
body1 = rigidBody('body1');
jnt1 = rigidBodyJoint('jnt1','revolute');
body2 = rigidBody('body2');
jnt2 = rigidBodyJoint('jnt2','revolute');
body3 = rigidBody('body3');
jnt3 = rigidBodyJoint('jnt3','prismatic');
body4 = rigidBody('body4');
jnt4 = rigidBodyJoint('jnt4','prismatic');
body5 = rigidBody('body5');
jnt5 = rigidBodyJoint('jnt5','revolute');

setFixedTransform(jnt1,dhparams(1,:),'dh');
setFixedTransform(jnt2,dhparams(2,:),'dh');
setFixedTransform(jnt3,dhparams(3,:),'dh');
setFixedTransform(jnt4,dhparams(4,:),'dh');
setFixedTransform(jnt5,dhparams(5,:),'dh');

body1.Joint = jnt1;
body2.Joint = jnt2;
body3.Joint = jnt3;
body4.Joint = jnt4;
body5.Joint = jnt5;
addBody(robot,body1,'base')
addBody(robot,body2,'body1')
addBody(robot,body3,'body2')
addBody(robot,body4,'body3')
addBody(robot,body5,'body4')
%%
showdetails(robot)
q0 = homeConfiguration(robot);
ndof = length(q0);
qs = zeros(1, ndof);
%%
figure(1)
q1=deg2rad(0);
q2=deg2rad(0);
q3=0.17;
show(robot,qs');