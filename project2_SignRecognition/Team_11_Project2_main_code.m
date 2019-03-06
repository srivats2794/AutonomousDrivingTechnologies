% Project 2 Main Code
% This code is an extension of the project 1 main code containing the same
% functionality, but with the addition of opening communication with the
% sign recognition laptop, slowing or stopping depending on the signal
% received
% 4/21/18
% Team 11

clear;
clc;
close all;

% define camera
cam = webcam('USB_camera');
% Permissible resolutions
% '640x480', '160x120', '176x144', '320x240', '352x288', '800x600', '1280x720', '1920x1080'
cam.resolution = '320x240';

r=1; % set rescale value
pic = snapshot(cam); % take snapshot
pic = imresize(pic,r); % resize
shape = size(pic); % store size of image

% initiate arduino connection and define pins
a = arduino('COM9', 'Uno', 'Libraries', 'Servo');
v = servo(a,'D11', 'MinPulseDuration', 1e-3, 'MaxPulseDuration',2e-3);
s = servo(a,'D10', 'MinPulseDuration', 1e-3, 'MaxPulseDuration',2e-3);

% set velocity and steering to neutral
writePosition(v, 0.5);
writePosition(s, 0.5);
pause(0.05);


% covariance and state variables for slope and intercept Kalman
Q=eye(4)*1000; R=eye(4)*10;%
y1=[]; P1=zeros(4);

% Initialize lane parameters, straight lanes are used so the vehicle tracks
% straight
o=0.3;
mr=-999;br=-mr*(1-o)*shape(2);
ml=999;bl=-ml*o*shape(2);
rtX=[(1-o)*shape(2) (shape(1)-br)/mr]; ltX=[o*shape(2) (shape(1)-bl)/ml];

% initiate steering angle and steering covariance
steer_ang=0.5; P_final=0;

%Setting up connection
ipA = '198.21.210.246'; portA= 9090; % sign recognition laptop
ipB= '198.21.191.63'; portB= 9091; % vehicle control laptop
udpB= udp(ipA, portA, 'LocalPort', portB);
fopen(udpB)

% Initialize various count variables
gcount = 0; % global count (iterates each loop)
count=0; % count for velocity oscillations
stopcount = 0; % count for stop time
slowcount = 0; % count for slowing
slowstart = 0; % variable for storing the global count at start of slowing
stopstart = -350; % variable for storing the global count at start of stopping
ML_Info = 0; % signal variable from sign recognition laptop

dateiname = ['lane_rec_',datestr(now,'yyyy-mm-dd'), '_', datestr(now,'HH-MM'),'.avi'];
v = VideoWriter(dateiname);
open(v);

H=figure(1); % store figure handle

slopes = []; intercepts = [];
str_angs=[]; dev_ds=[]; dev_as=[];
while ishandle(H) % while the figure is open
    
    %t0=tic; % timer
    
    pic = snapshot(cam); % take snapshot
    pic = imresize(pic,r); % resize
    
    % run camera function
    [mr,br,ml,bl,y1,P1,rtX,ltX]=Team_11_lane_recognition(pic,Q,R,shape,y1,P1,mr,br,ml,bl,rtX,ltX);
    
    frame = getframe(gcf);
    writeVideo(v,frame);
    
    slopes=[slopes; mr, ml];
    intercepts=[intercepts; br, bl];
    
    % run steering control function
    [steer_ang,dev_dist,dev_angle] = Team_11_Steer_Control_Function(y1(3),y1(4),y1(1),y1(2),0.7*shape(1),shape(1),shape(2));
    
    str_angs=[str_angs;steer_ang];
    dev_ds=[dev_ds;dev_dist];
    dev_as=[dev_as;dev_angle];
    
    %Accepting ML Data from second laptop
    % check for signals if more than 400 iterations after stopping
    % otherwise flush the signal
    if (gcount-stopstart)>400
        % read signal and flush only if signal is available
        if udpB.BytesAvailable > 0
            ML_Info = fread(udpB, udpB.BytesAvailable);
            flushinput(udpB);
        end
    else
        flushinput(udpB);
    end
    
    % if stop signal received increment stop counter store the start count
    % and set slow count to 0
    if ML_Info == 2    
        stopcount = stopcount + 1;
        stopstart = gcount;
        slowcount = 0;
        disp('stop sign observed')
    end
    
    % if slow signal received increment slow count and store start count
    if ML_Info == 1
        slowcount = slowcount + 1;
        slowstart = gcount;
        disp('school zone')
    end
    
    % if not stopped and slowcount non-zero increment slowcount and adjust
    % speed
    if slowcount>0 && ML_Info ~= 2
        slowcount = slowcount + 1;
        speed=0.56935;
        disp(0.56935)
    end
    
    % if stop count between 0 and 100 stop, increment stop count and set
    % ML_Info to 0; otherwise reset stop count
    if stopcount<100 && stopcount>0
        speed= 0.5;
        stopcount = stopcount+1;
        ML_Info = 0;
        disp('stopped')
    else
        stopcount = 0;
    end
                    
        
    % if not stopped or slowed proceed at higher speed
    if (stopcount>=100 || stopcount<=0) && slowcount == 0
        % Slow speed for 25 iterations, high speed for 5, reset counter 
        % after 30 loops
        if count<5
            count=count+1;
            speed=0.5795;
        elseif count>=5 && count<=30
            count=count+1;
            speed=0.56945;
        else
            count=0;
            speed=0.56945;
        end
        
        % Average speed
        disp(0.5795*5/31+0.56945*26/31)
        
    end
    % send velocity and steering command to the vehicle
    writePosition(v, speed);
    writePosition(s, steer_ang);
    gcount = gcount+1;
    pause(.001)
    
    %toc(t0);% end timer
    
end

% set velocity and steering to neutral
writePosition(v, 0.5);
writePosition(s, 0.5);
pause(.05)

fclose(instrfindall); % close the connection

close(v);
save('inputs','slopes','intercepts','str_angs','dev_ds','dev_as')
