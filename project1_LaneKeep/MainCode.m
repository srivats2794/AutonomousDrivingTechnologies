clear;
clc;
close all;

% define camera
cam = webcam('USB_camera');
% '640x480', '160x120', '176x144', '320x240', '352x288', '800x600', '1280x720', '1920x1080'
cam.resolution = '320x240';

% set rescale value
r=1;
pic = snapshot(cam); % take snapshot
pic = imresize(pic,r); % resize
shape = size(pic); % store size of image

% initiate arduino connection and define pins
a = arduino('COM3', 'Uno', 'Libraries', 'Servo');
v = servo(a,'D11', 'MinPulseDuration', 1e-3, 'MaxPulseDuration',2e-3);
s = servo(a,'D10', 'MinPulseDuration', 1e-3, 'MaxPulseDuration',2e-3);

% set velocity and steering to neutral
writePosition(v, 0.5);
writePosition(s, 0.5);
pause(0.05);


% covariance and state varaibles for slope and intercept Kalman
Q=eye(4)*1000; R=eye(4)*10;% diag([0.0105^2 9.6749^2 0.033^2 16.8557^2]);
y1=[]; P1=zeros(4); 
o=0.3;
mr=-999;br=-mr*(1-o)*shape(2);
ml=999;bl=-ml*o*shape(2);
rtX=[(1-o)*shape(2) (shape(1)-br)/mr]; ltX=[o*shape(2) (shape(1)-bl)/ml];
% initiate steering angle and steering covariance
steer_ang=0.5; P_final=0;

H=figure(1); % store figure handle

% vel=0.569;
% writePosition(v, vel);
% pause(0.05)
count=0;
while ishandle(H) % while the figure is open
    t0=tic;
    
    pic = snapshot(cam); % take snapshot
    pic = imresize(pic,r); % resize
    
    % run camera function
    [mr,br,ml,bl,y1,P1,rtX,ltX]=cp002(pic,Q,R,shape,y1,P1,mr,br,ml,bl,rtX,ltX);
    
    % run steering control function
%     [steer_ang,P_final] = Steer_Control_Function_Kalman(y1(3),y1(4),y1(1),y1(2),0.5*shape(1),shape(1),shape(2),steer_ang,P_final);
    [steer_ang] = Steer_Control_Function(y1(3),y1(4),y1(1),y1(2),0.7*shape(1),shape(1),shape(2));
    
    % generate velocity based on steering input
%     vel=0.57;
%     k=abs(steer_ang-0.5);
%     if k>=0.1 && k<=0.3
%         vel=0.569;
%     elseif k>0.3 && k<=0.5
%         vel=0.568;
%     end
    
    % send velocity and steering command to the vehicle
    if count<25
        count=count+1;
        writePosition(v, 0.5635);
    elseif count>=25 && count<=30
        count=count+1;
        writePosition(v, 0.56945);
    else
        count=0;
        writePosition(v, 0.5635);
    end
    writePosition(s, steer_ang);
    pause(.001)
    
    toc(t0);
end

% set velocity and steering to neutral
writePosition(v, 0.5);
writePosition(s, 0.5);
pause(.05)
