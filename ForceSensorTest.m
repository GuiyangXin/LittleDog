clear;clc;close all
r = LittleDog;
sys = TimeSteppingRigidBodyManipulator(r,0.001);
sys = addSensor(sys,FullStateFeedbackSensor());
body = findLinkId(sys,'front_right_lower_leg');

% Put force sensor on different position of the front_right_lower_leg
% frame = RigidBodyFrame(body,zeros(3,1),zeros(3,1),'FT_frame');
frame = RigidBodyFrame(body,[-0.0265 0 -0.0985]',zeros(3,1),'FT_frame');

sys = addFrame(sys,frame);
sys = addSensor(sys,ContactForceTorqueSensor(sys,frame));
sys = compile(sys);
v = constructVisualizer(sys);
sys1=sys;

foot = struct('id',[],'in_stance',[]);
foot(1).id = sys.findFrameId('front_left_foot_center');
foot(2).id = sys.findFrameId('front_right_foot_center');
foot(3).id = sys.findFrameId('back_left_foot_center');
foot(4).id = sys.findFrameId('back_right_foot_center');
body_id=findLinkId(sys,'body');
FT_frame_id=sys.findFrameId('FT_frame');
    
x0 = home(r);
qa0 = x0(getActuatedJoints(r));
nq = r.getNumPositions();
qstar = x0(1:nq);
v.draw(0,qstar);

hip_roll = 0.1;
hip_pitch = 1;
knee = 1.55;
xstart = Point(getStateFrame(sys));
xstart.front_right_hip_roll = -hip_roll;
xstart.front_right_hip_pitch = hip_pitch;
xstart.front_right_knee = -knee;
xstart.front_left_hip_roll = hip_roll;
xstart.front_left_hip_pitch = hip_pitch;
xstart.front_left_knee = -knee;
xstart.back_right_hip_roll = -hip_roll;
xstart.back_right_hip_pitch = -hip_pitch;
xstart.back_right_knee = knee;
xstart.back_left_hip_roll = hip_roll;
xstart.back_left_hip_pitch = -hip_pitch;
xstart.back_left_knee = knee;
xstart.base_z = 0.146;

T=2;
      
% construct PD control 
Kp = 5*eye(12);
Kd = diag([0.5; 0.6; 0.16; 0.5; 0.6; 0.16; 0.5; 0.5; 0.16; 0.5; 0.5; 0.16]);
sys = pdcontrol(sys,Kp,Kd);
      
% send position reference
sys = cascade(setOutputFrame(ConstOrPassthroughSystem(qa0),getInputFrame(sys)),sys);

if (0)
    sys = cascade(sys,v);
    simulate(sys,[0 2],double(xstart));
else
    [xtraj,ytraj] = simulate(sys,[0 2],double(xstart));
    playback(v,xtraj,struct('slider',true));
    figure
    plot(xtraj.xx(37,:))
    figure
    plot(xtraj.xx(38,:))
    figure
    plot(xtraj.xx(39,:))
    figure
    plot(xtraj.xx(40,:))
    figure
    plot(xtraj.xx(41,:))
    figure
    plot(xtraj.xx(42,:))
end

% Convert the result into world frame
options.rotation_type=2;
for i=2:T/0.001+1
    kinsol = doKinematics(sys1,xtraj.xx(1:18,i-1));
    pose=zeros(3,4);
    pose= forwardKin(sys1,kinsol,FT_frame_id,[0;0;0],options);
    quat = pose(4:7);
    rotm = quat2rotm(quat);
    force(:,i-1)=rotm*xtraj.xx(37:39,i-1);
end
figure
plot(force(1,:))
figure
plot(force(2,:))
figure
plot(force(3,:))