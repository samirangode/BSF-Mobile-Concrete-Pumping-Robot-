body1 = robotics.RigidBody('body1');
jnt1 = robotics.Joint('jnt1','revolute');
jnt1.HomePosition = 0;
tform = trvec2tform([0, 0, 0]); % User defined
setFixedTransform(jnt1,tform);
body1.Joint = jnt1;

robot = robotics.RigidBodyTree;

addBody(robot,body1,'base');

body2 = robotics.RigidBody('body2');
jnt2 = robotics.Joint('jnt2','revolute');
jnt2.HomePosition = 0; % User defined
tform2 = trvec2tform([8.5, 0, 0]); % User defined
setFixedTransform(jnt2,tform2);
body2.Joint = jnt2;
addBody(robot,body2,'body1'); % Add body2 to body1

body3 = robotics.RigidBody('body3');
jnt3 = robotics.Joint('jnt3','revolute');
tform3 = trvec2tform([7.5, 0, 0]); % User defined
setFixedTransform(jnt3,tform3);
body3.Joint = jnt3;
addBody(robot,body3,'body2');

body4 = robotics.RigidBody('body4');
jnt4 = robotics.Joint('jnt4','revolute');
tform4 = trvec2tform([7.5, 0, 0]); % User defined
body4.Joint = jnt4;
addBody(robot,body4,'body3');

body = robotics.RigidBody('tool');
joint = robotics.Joint('fix1','revolute');
setFixedTransform(joint, trvec2tform([7.5, 0, 0]));
body.Joint = joint;
addBody(robot,body,'body4');

showdetails(robot);