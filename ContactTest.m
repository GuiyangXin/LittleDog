clear;clc;close all
r = LittleDog;
sys = TimeSteppingRigidBodyManipulator(r,0.001);
sys = addSensor(sys,FullStateFeedbackSensor());
body = findLinkId(sys,'front_right_lower_leg');
% frame = RigidBodyFrame(body,zeros(3,1),zeros(3,1),'FT_frame');
frame = RigidBodyFrame(body,[-0.0265 0 -0.0985]',zeros(3,1),'FT_frame');
sys = addFrame(sys,frame);
sys = addSensor(sys,ContactForceTorqueSensor(sys,frame));
sys = compile(sys);
v = constructVisualizer(sys);
sys1=sys;

foot = struct('id',[],'in_stance',[]);
foot(1).id = r.findFrameId('front_left_foot_center');
foot(2).id = r.findFrameId('front_right_foot_center');
foot(3).id = r.findFrameId('back_left_foot_center');
foot(4).id = r.findFrameId('back_right_foot_center');
body_id=findLinkId(r,'body');
FTid=sys.findFrameId('FT_frame');
    
x0 = home(r);
% x0 = r.resolveConstraints(x0);
qa0 = x0(getActuatedJoints(r));
nq = r.getNumPositions();
qstar = x0(1:nq);
v.draw(0,qstar);
% x_force0=initialize(sys);
hip_roll = 0.1;
hip_pitch = 1;
knee = 1.55;
x_force0 = Point(getStateFrame(sys));
x_force0.front_right_hip_roll = -hip_roll;
x_force0.front_right_hip_pitch = hip_pitch;
x_force0.front_right_knee = -knee;
x_force0.front_left_hip_roll = hip_roll;
x_force0.front_left_hip_pitch = hip_pitch;
x_force0.front_left_knee = -knee;
x_force0.back_right_hip_roll = -hip_roll;
x_force0.back_right_hip_pitch = -hip_pitch;
x_force0.back_right_knee = knee;
x_force0.back_left_hip_roll = hip_roll;
x_force0.back_left_hip_pitch = -hip_pitch;
x_force0.back_left_knee = knee;
x_force0.base_z = 0.146;

T=2;
deltat=0.001;
t=0:0.001:T;
q=qstar;
Jx=[eye(6,6) zeros(6,12)];

options.rotation_type=0;
options.in_terms_of_qdot=true;
options.base_or_frame_id=findLinkId(r,'world');
for i=2:T/0.001+1
    kinsol = doKinematics(r,q(:,i-1));
    x=zeros(3,4);
    Jc=zeros(12,18);
    for j=1:4
        [x(:,j),Jc(3*j-2:3*j,:)] = forwardKin(r,kinsol,foot(j).id,[0;0;0],options);
    end
%     [xb,Jcb] = forwardKin(r,kinsol,body_id,[0;0;0],options);
    I=eye(18,18);
    P=I-pinv(Jc)*Jc;

    Jb=Jx*P;
    [U,S,V]=svd(Jb);
    T=zeros(6,18);
    for k=1:6
        T(k,k)=1/S(k,k);
    end
    Jbinv=V*T'*U';
    Jbinv=[eye(6,6);Jbinv(7:18,:)];
    
    if i<=1001
        vb=[0 0.05/1-(0.05/1)*cos(2*pi*t(i)/1) 0 0 0 0]';
        vq(:,i)=Jbinv*vb;
        q(:,i)=vq(:,i)*deltat+q(:,i-1);
        aq(:,i)=(vq(:,i)-vq(:,i-1))/deltat;
    else
        vb=[0 -0.05/1+(0.05/1)*cos(2*pi*t(i)/1) 0 0 0 0]';
        vq(:,i)=Jbinv*vb;
        q(:,i)=vq(:,i)*deltat+q(:,i-1);
        aq(:,i)=(vq(:,i)-vq(:,i-1))/deltat;
    end

%     vb=[0 (0.05*pi*cos((pi/3)*t(i-1)))/3 0 0 0 0]';
%     vq(:,i)=Jbinv*vb;
%     q(:,i)=vq(:,i)*deltat+q(:,i-1);
%     aq(:,i)=(vq(:,i)-vq(:,i-1))/deltat;
end

qa=q(7:18,:);
qdes_traj = PPTrajectory(spline(t,qa));
      
% construct PD control 
Kp = 5*eye(12);
Kd = diag([0.5; 0.6; 0.16; 0.5; 0.6; 0.16; 0.5; 0.5; 0.16; 0.5; 0.5; 0.16]);
sys = pdcontrol(sys,Kp,Kd);
      
% send position reference
sys = cascade(setOutputFrame(qdes_traj,getInputFrame(sys)),sys);
% sys = cascade(setOutputFrame(ConstOrPassthroughSystem(qa0),getInputFrame(sys)),sys);

if (0)
    sys = cascade(sys,v);
    simulate(sys,[0 2],double(x_force0));
else
    [xtraj,ytraj] = simulate(sys,[0 2],double(x_force0));
    playback(v,xtraj,struct('slider',true));
    figure
    plot(xtraj.xx(39,:))
    figure
    plot(xtraj.xx(40,:))
    figure
    plot(xtraj.xx(41,:))
    figure
    plot(xtraj.xx(42,:))
%     figure
%     plot(xtraj.xx(1,:))
%     hold on
%     plot(q(1,:),'r')
%     figure
%     plot(xtraj.xx(2,:))
%     hold on
%     plot(q(2,:),'r')
%     figure
%     plot(xtraj.xx(3,:))
%     hold on 
%     plot(q(3,:),'r')
end

%parse the measurement result
% q1=qstar;
% options.rotation_type=1;
% kinsol = doKinematics(sys1,q1);
% pose=forwardKin(sys1,kinsol,FTid,[0;0;0],options);
options.rotation_type=2;
% pose1=forwardKin(sys1,kinsol,foot(1).id,[0;0;0],options);
% pose2=forwardKin(sys1,kinsol,foot(2).id,[0;0;0],options);
% quat = pose1(4:7);
% rotm = quat2rotmat(quat);
for i=2:T/0.001+1
    kinsol = doKinematics(sys1,q(:,i-1));
    pose=zeros(3,4);
    pose= forwardKin(sys1,kinsol,FTid,[0;0;0],options);
    quat = pose(4:7);
    rotm = quat2rotmat(quat);
    force(:,i-1)=rotm*xtraj.xx(37:39,i-1);
end
figure
plot(force(1,:))
figure
plot(force(2,:))
figure
plot(force(3,:))