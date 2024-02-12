clc; clear all; close all; % Default setting

% vid_name   = 'two_trajectory_planar_robot';
% v = VideoWriter(vid_name,'MPEG-4');
% open(v);                 % Make video as .mp4

radius = 8;                % trajectory's radius = 10
theta = 0:0.01:2*pi;       % range = [0,2*pi]

% theta = flip(theta);     % counter rotation

trajX = radius*cos(theta); % trajectory's x element
trajY = radius*sin(theta); % trajectory's y element

plot(trajX,trajY,'b-');    % trajectory generation
axis([-10 10 -10 10]);
axis equal;
hold on; grid on;
l1 = 5;                    % link 1's length
l2 = 5;                    % link 2's length

q1_init = 0;               % initial angle
q2_init = 0;               % initial angle

[px1_init,py1_init,px2_init,py2_init] = two_link_FK(l1,l2,q1_init,q2_init);

joint2_hlr = plot(px2_init,py2_init,'k*');                            % joint 2 Forward Kinematics
joint1_hlr = plot(px1_init,py1_init,'k*');                            % joint 1 Forward Kinematics
base   = plot(0,0,'ko');                                              % fixed base position

linkX_init = [0,px1_init,px2_init];
linkY_init = [0,py1_init,py2_init];
link_hlr = plot(linkX_init,linkY_init,'k-');                          % link line drawing 

xlabel('Pos X [m]');
ylabel('Pos Y [m]');                                                  % labeling to axis

for i= 1:4:length(theta)                                              % iteration
    str = sprintf('Two Link Planar Robot Circle Trajectory');
    title(str);

    % IK => FK : find angle commands and apply them into each joints(motors)
    
    [q1_IK,q2_IK] = two_link_IK(radius,l1,l2,trajX(i),trajY(i));      % Inverse Kinematics
    [px1,py1,px2,py2]= two_link_FK(l1,l2,q1_IK,q2_IK);                % Forward Kinematics
    linkX = [0,px1,px2];
    linkY = [0,py1,py2];

    set(joint2_hlr,'XData',px2,'YData',py2);                          
    set(joint1_hlr,'XData',px1,'YData',py1);                            
    set(link_hlr,'XData',linkX,'YData',linkY);                        % Update datas

    % frame = getframe(gcf);
    % writeVideo(v,frame);                                            % Snap Each iteration into video

    drawnow;
    pause(0.01);
end

% close(v);                                                           % close video and save it

function [px1,py1,px2,py2]= two_link_FK(l1,l2,q1_FK,q2_FK)            % Forward Kinematics
    px2 = l1*cos(q1_FK) + l2*cos(q1_FK+q2_FK);
    py2 = l1*sin(q1_FK) + l2*sin(q1_FK+q2_FK);    
    px1 = l1*cos(q1_FK);
    py1 = l1*sin(q1_FK);
end

function [q1_IK,q2_IK] = two_link_IK(radius,l1,l2,px,py)              % Inverse Kinematics
    c2 = (radius^2 - l1^2 - l2^2)/(2*l1*l2); % px2^2 + py2^2 = r^2
    s2 = sqrt(1-c2^2);
    
    q2_IK = atan2(s2,c2);

    q1_IK = atan2(py,px) - atan2(l2*sin(q2_IK),l1+l2*cos(q2_IK));
end
