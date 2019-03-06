% Project 2 Sign Recognition
% This code opens a communication connection with the vehicle control
% laptop and sign recognition camera. It then detects school signs and stop
% signs and then sends the signal to the vehicle control laptop
% 4/21/18
% Team 11

clc;
clear;
close all;

%Setting up connection
ipA= '198.21.207.116';   portA= 9090; % sign recognition laptop
ipB= '198.21.191.63';  portB= 9091; % vehicle control laptop
udpA= udp(ipB,portB,'LocalPort',portA);
fopen(udpA);

% define camera
cam = webcam('USB2.0 PC CAMERA'); 
cam.resolution = '320x240';

% Initialize various count variables
count=0; % school sign count
count2=0; % stop sign count
gcount=0; % global count

H=figure(1);% store figure handle

dateiname = ['sign_rec_',datestr(now,'yyyy-mm-dd'), '_', datestr(now,'HH-MM'),'.avi'];
v = VideoWriter(dateiname);
open(v);

while ishandle(H)% while the figure is open
    
    %t0=tic; % timer
    
    %% image processing
    image = snapshot(cam); % take snapshot
    imagegray=rgb2gray(image); % transfer to grey
    
    % detect school sign
    detector = vision.CascadeObjectDetector('school_sign_file.xml');
    bbox= step(detector,imagegray);
    
    % detect stop sign
    detector2 = vision.CascadeObjectDetector('stop_sign_file.xml');
    bbox2= step(detector2,imagegray);

    %% School sign algorithm
    if ~isempty(bbox) % if school sign detected
        if count==0 % if it is the first detection store global count
            startc=gcount;
        end
        count=count+1; % increment school sign count  
    end 
    
    if exist('startc','var')
        % if 2 school signs are detected within 5 iterations set signal to
        % 1, if more than 5 iterations since first sighting reset counters
        % and set signal to 0, otherwise set signal to 0
        if count==2 && gcount-startc<=5
            A=1;
            count=0;
        elseif gcount-startc>5
            startc=0; count=0;
            A=0;
        else 
            A=0;
        end
    else 
        A=0;
    end    
    
    %%    Stop sign algorithm
    
    if ~isempty(bbox2) % if stop sign detected
        if count2==0 % if it is the first detection store global count
            startc2=gcount;
        end
        count2=count2+1; % increment stop sign count  
    end 
    if exist('startc2','var') 
        % if 12 stop signs are detected within 20 iterations set signal to
        % 2, if more than 20 iterations since first sighting reset counters
        % and maintain signal from school sign check
        if count2==12 && gcount-startc2<20
            A=2;
            count2=0;
        elseif gcount-startc2>20
            startc2=0; count2=0;
        end
    end
    
    if A ~= 0 % if signal not 0 send to control laptop
        fwrite(udpA,A);
    end
    
  %% BBOX display images
    detectedImg= insertObjectAnnotation(image,'rectangle',bbox,'school_sign');
    detectedImg2= insertObjectAnnotation(image,'rectangle',bbox2,'stop_sign');
    subplot(1,2,1), imshow(detectedImg)
    subplot(1,2,2), imshow(detectedImg2)
    
    frame = getframe(gcf);
    writeVideo(v,frame);
    
    gcount=gcount+1; % increment global count
    
    %toc(t0)% end timer
    
end

fclose(instrfindall); % close the connection

close(v);