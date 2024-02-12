clc; clear all; close all; % Default setting

% vid_name   = 'Three Link Planar Robot Circle Trajectory';
% v = VideoWriter(vid_name,'MPEG-4');
% open(v);                 % Make video as .mp4

radius = 3;                % trajectory's radius = 3
theta = 0:0.01:2*pi;       % range = [0,2*pi]

% theta = flip(theta);     % counter rotation

trajX = 5 + radius*cos(theta); % trajectory's x element
trajY = 7 + radius*sin(theta); % trajectory's y element

plot(trajX,trajY,'b--');   % trajectory generation
axis([-12 12 -12 12]);
axis equal;
hold on; grid on;
l1 = 5;                    % link 1 length
l2 = 4;                    % link 2 length
l3 = 3;                    % link 3 length

q1_init = 0;            % q1 initial angle
q2_init = 0;            % q2 initial angle
q3_init = 0;            % q3 initial angle
phi = pi/4;             % angle constraint for singularity avoidance

[px1_init,py1_init,px2_init,py2_init,px3_init,py3_init] = three_link_FK(l1,l2,l3,q1_init,q2_init,q3_init);
joint3_hlr = plot(px3_init,py3_init,'k*');                            % joint 3 initial Forward Kinematics
joint2_hlr = plot(px2_init,py2_init,'k*');                            % joint 2 initial Forward Kinematics
joint1_hlr = plot(px1_init,py1_init,'k*');                            % joint 1 initial Forward Kinematics
base   = plot(0,0,'ko');                                              % fixed base position

linkX_init = [0,px1_init,px2_init,px3_init];
linkY_init = [0,py1_init,py2_init,py3_init];
link_hlr = plot(linkX_init,linkY_init,'k-');                          % link line drawing 

xlabel('Pos X [m]');
ylabel('Pos Y [m]');                                                  % labeling to axis

for i= 1:4:length(theta)                                              % iteration
    str = sprintf('Three Link Planar Robot Circle Trajectory');
    title(str);

    % IK => FK : find angle commands and apply them into each joints(motors)

    [q1_IK,q2_IK,q3_IK] = three_link_IK(l1,l2,l3,phi,trajX(i),trajY(i));         % Inverse Kinematics
    [px1,py1,px2,py2,px3,py3] = three_link_FK(l1,l2,l3,q1_IK,q2_IK,q3_IK);       % Forward Kinematics
    linkX = [0,px1,px2,px3];
    linkY = [0,py1,py2,py3];

    set(joint3_hlr,'XData',px3,'YData',py3);
    set(joint2_hlr,'XData',px2,'YData',py2);                          
    set(joint1_hlr,'XData',px1,'YData',py1);                            
    set(link_hlr,'XData',linkX,'YData',linkY);                        % Update datas
    
    % frame = getframe(gcf);
    % writeVideo(v,frame);                                            % Snap Each iteration into video

    drawnow;
    pause(0.01);
end

% close(v);                                                           % close video and save it

function [px1,py1,px2,py2,px3,py3]= three_link_FK(l1,l2,l3,q1_FK,q2_FK,q3_FK) % Forward Kinematics
    px3 = l1*cos(q1_FK) + l2*cos(q1_FK+q2_FK) + l3*cos(q1_FK+q2_FK+q3_FK);
    py3 = l1*sin(q1_FK) + l2*sin(q1_FK+q2_FK) + l3*sin(q1_FK+q2_FK+q3_FK);
    px2 = l1*cos(q1_FK) + l2*cos(q1_FK+q2_FK);
    py2 = l1*sin(q1_FK) + l2*sin(q1_FK+q2_FK);    
    px1 = l1*cos(q1_FK);
    py1 = l1*sin(q1_FK);
end

function [q1_IK,q2_IK,q3_IK] = three_link_IK(l1,l2,l3,phi,px,py)              % Inverse Kinematics
    px = px - l3*cos(phi);
    py = py - l3*sin(phi);

    C2 = (px^2 + py^2 - (l1^2 + l2^2)) / (2*l1*l2);
    q2_IK = atan2(sqrt(1-C2^2) , C2);

    C1 = ((l1 + l2*C2)*px + l2*sqrt(1-C2^2)*py) / (px^2 + py^2);
    S1 = ((l1 + l2*C2)*py - l2*sqrt(1-C2^2)*px) / (px^2 + py^2);
    q1_IK = atan2(S1,C1);

    q3_IK = phi - q1_IK - q2_IK;
end