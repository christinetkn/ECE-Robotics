%% Robotics-I 2020-2021
%% AM: 03117012
%% Simulation Part
clear; % make sure previously defined variables are erased.
clc

l = zeros(1,5);
l(2) = 20.0;
l(4) = 15.0;
l(5) = 10.0;

% sampling period for the robot motion
% we set the duration of motion 20 seconds

dt = 0.002;
Tf = 20.0;
t = 0:dt:Tf;
  
%%  DESIRED MOTION PROFILE  %%
% The linear movement will  be executed in height h above z0
%we chose h to be 25cm
h = 25.0;
space_length = 18.0;

% p_A : initial position of end-effector
% v_A : initial velocity of end-effector
% v_B : final velocity of end-effector
p_A = [0 0 h];
p_B = [18 0 h];
v_A = 0.0;
v_B = 0.0;

pe_x(1) = p_A(1);
pe_y(1) = p_A(2);
pe_z(1) = p_A(3);

% the number of points we used
points = Tf/dt + 1;

%space will represent 0ex
space = zeros (1, points);
v = zeros (1, points); %linear speed of end-effector

space(1) = 0;
v(1) = v_A;

for k = 2:points
    disp(space(k))
    if ((space(k-1) >= 0) && (space(k-1) < 4 ))
        acceleration = 0.5;
    elseif ((space(k-1) >= 4) && (space(k-1) < 8))
        acceleration = 0;
    else
        acceleration = -0.5;   
    end

    v(k) = v(k-1) + acceleration*(dt);
    space(k) = space(k-1) + v(k)*dt;
    
    pe_y(k) = pe_y(1);
    pe_z(k) = pe_z(1);
end

v_x(1) = 0;
v_y(1) = 0;
v_z(1) = 0;
v_x(2:points) = diff(space)/dt;
v_y(2:points) = diff(pe_y)/dt;
v_z(2:points) = diff(pe_z)/dt;


%% using Inverse Kinematics   %%
%% compute the reference joint-motion vectors: qd(k,i) and velocities qd_1(k,i)%%

qd = zeros(points,3);
qd(:,3) = acos((pe_x(:).^2 + pe_y(:).^2 + pe_z(:).^2 - l(2)^2 - l(4)^2 - l(5)^2)./(2*l(4)*l(5)));
s3 = sin(qd(:,3));
c3 = cos(qd(:,3));
qd(:,2) = acos((l(5).*s3.*pe_y(:)+(l(4) +l(5).*c3).*sqrt((l(4) +l(5).*c3).^2 +l(5)^2.*s3.^2 - pe_y(:).^2)) ./(l(5)^2.*s3.^2 + (l(4)+l(5).*c3).^2))
s2 = sin(qd(:,2));
c2 = cos(qd(:,2));
q23 = qd(:,3) + qd(:,2);
s23 = sin(q23);
c23 = cos(q23);
qd(:,1) = acos(((l(4)+c2+l(5).*c23).*pe_z(:)+l(2).*sqrt((l(4).*c2 +l(5).*c23).^2 + (l(1)+l(2)).^2 - pe_z(:).^2)) ./((l(4).*c2 + l(5).*c23).^2+l(2)^2))
s1 = sin(qd(:,1));
c1 = cos(qd(:,1));

detJ = l(4)*l(5).*s3.*(c2*l(4)+c23*l(5));

J11 = -(c1.*s3*l(4)*l(5))./detJ;
J12 = l(1)./detJ;
J13 = s1.*s3*l(4)*l(5)./detJ;
J21 = l(5)*c23.*(l(2)*c1 + s1.*c2*l(4) + s1.*c23*l(5))./detJ;
J22 = l(5)*s23.*(l(4)*c2+l(5)*c23)./detJ;
J23 = l(5)*s23.*(l(2).*(-s1)+c1.*c2*l(4)+c1.*c23*l(5))./detJ;
J31 = -(l(2)*c1 + s1.*c2*l(4)+s1.*c23*l(5)).*(c2*l(4)+c23*l(5))./detJ;
J32 = -(s2*l(4)+s23*l(5)).*(c2*l(4)+c23*l(5))./detJ;
J33 = (c2*l(4)+c23*l(5)).*((-s1)+c1.*c2*l(4)+c1.*c23*l(5))./detJ;

vqd = ones(points, 3);
for k = 1:points
    inv_Jp = [J11(k) J12(k) J13(k); J21(k) J22(k) J23(k); J31(k) J32(k) J33(k)];
    ve = [v_x(k) v_y(k) v_z(k)];
    %we = [w_x(k) w_y(k) w_z(k)];
    vqd(k,:) = inv_Jp * ve';
end


%% by using Forward Kinematics we compute  the  Joint Motion  %%
% (p1x, p1y, p1z) : cartesian position of the 1st link's frame
p1x = zeros(1,points);
p1y = zeros(1,points);
p1z = zeros(1,points);
% (p2x, p2y, p2z) : cartesian position of the 2nd link's frame
p2x = c2.*s1*l(4);
p2y = l(4)*s2;
p2z = -c1.*c2*l(4);
% (p3x, p3y, p3z) : cartesian position of the 3rd link's frame
p3x = l(1)+c1.*l(2)+s1.*c23*l(5)+s1.*c2*l(4);
p3y = s23*l(5)+s2*l(4);
p3z = s1*l(2)-c1.*c2*l(4)-c1.*c23*l(5);


figure('Name','End-effector trajectory','NumberTitle','off');
subplot(3,1,1); 
plot(t,space); 
ylabel('pe_x (cm)'); 
xlabel('time t (sec)');  

subplot(3,1,2); 
plot(t,pe_y); 
ylabel('pe_y (cm)'); 
xlabel('time t (sec)');  

subplot(3,1,3); 
plot(t,pe_z); 
ylabel('pe_z (cm)'); 
xlabel('time t (sec)');  

figure('Name','End-effector velocity ','NumberTitle','off');
subplot(2,1,1);
plot(t,v);
ylabel('v (cm)');
xlabel('time t (sec)');  

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('Name','Joints angles','NumberTitle','off');
subplot(3,1,1); 
plot(t,qd(:,1)); 
ylabel('vqd1 (rad)'); 
xlabel('time t (sec)');  

subplot(3,1,2); 
plot(t,qd(:,2)); 
ylabel('vqd2 (rad)'); 
xlabel('time t (sec)');    

subplot(3,1,3); 
plot(t,qd(:,3)); 
ylabel('vqd3 (rad)'); 
xlabel('time t (sec)');    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('Name','Joints angular velocity','NumberTitle','off');
subplot(3,1,1); 
plot(t,vqd(:,1)); 
ylabel('qd1_1 (rad)'); 
xlabel('time t (sec)');  

subplot(3,1,2); 
plot(t,vqd(:,2)); 
ylabel('qd1_2 (rad)'); 
xlabel('time t (sec)');    

subplot(3,1,3); 
plot(t,vqd(:,3)); 
ylabel('qd1_3 (rad)'); 
xlabel('time t (sec)'); 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% animate robot motion
figure('Name','Animation','NumberTitle','off');
axis([-(l(2)) l(2)   0 h]) %% set xyz plot axes
axis on 
hold on 
xlabel('x (cm)'); 
ylabel('y (cm)');
zlabel('z (cm)');
plot3(space,pe_y,pe_z,'rs'); 
dtk=1000; %% plot robot position every dtk samples, to animate its motion 
plot3([0],[0],[0],'o');
for tk=1:dtk:points,
   pause(0.1); %% pause motion to view successive robot configurations    
   plot3([0,p1x(tk)],[0,p1y(tk)],[0,p1z(tk)]);                  
   plot3([p1x(tk)],[p1y(tk)],[p1z(tk)],'o');    
   plot3([p1x(tk),p2x(tk),p3x(tk)],[p1y(tk),p2y(tk),p3y(tk)],[p1z(tk),p2z(tk),p3z(tk)]);    
   plot3([p2x(tk)],[p2y(tk)],[p2z(tk)],'y*');    
   plot3([p3x(tk)],[p3y(tk)],[p3z(tk)],'mx'); 
   plot3([space(tk)],[pe_y(tk)],[pe_z(tk)],'g+');  
end
