function [rtm,rtb,ltm,ltb,y1,P1,rtX,ltX]=Team_11_lane_recognition(pic,Q,R,shape,y1,P1,rtm,rtb,ltm,ltb,rtX,ltX)
% this function takes a picture, finds the lane markers and outputs the
% respective slope and intercept of the left and right lanes within the
% pixel frame
% 4/10/18
% Team 11

% parameters for image masking
    target_x=0.1; target_y=0.2*shape(1); oset=0.9;
    a=[0,shape(2)*target_x,shape(2)*(1-target_x),shape(2),shape(2),0];% x coordinates of trapezoid
    b=[shape(1)*oset,target_y,target_y,shape(1)*oset,shape(1),shape(1)]; % y coordinates of trepezoid
    
    gray_pic = rgb2gray(pic); % gray scale
    edge_pic = edge(gray_pic,'canny',[0.1 0.6]); % find edges
        
    bw=roipoly(pic,a,b); % build masking area
    BW=(edge_pic(:,:,1)&bw); % combine masking with edge pic
    
% uncomment to plot picture
%     figure(1)
    hold off
    imshow(BW); hold on

    % perform hough transform of canny image
    [H,theta,rho]=hough(BW,'Theta',-45:45);
    % find the peaks
    P = houghpeaks(H,4,'Threshold',0.3*max(max(H)));
    % generate lines
    lines = houghlines(BW,theta,rho,P,'FillGap',10,'MinLength',7);
    
    % clear varaibles for loop
    rtpts1=[];rtpts2=[]; ltpts1=[];ltpts2=[];
    lttheta=[]; rttheta=[]; ltrho=[]; rtrho=[];
    
    for k = 1:length(lines) % loop through lines
% Uncomment to plot all lines found
%         xy = [lines(k).point1; lines(k).point2];
%         plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');
        
        % determine if the line is the left or right lane bound and store
        % in respective variables
        if lines(k).theta<0
            rtpts1=[rtpts1;lines(k).point1];
            rtpts2=[rtpts2;lines(k).point2];
            rttheta=[rttheta;lines(k).theta];
            rtrho=[rtrho;lines(k).rho];
        elseif lines(k).theta>=0
            ltpts1=[ltpts1;lines(k).point1];
            ltpts2=[ltpts2;lines(k).point2];
            lttheta=[lttheta;lines(k).theta];
            ltrho=[ltrho;lines(k).rho];
        end
    end
    
    % remove right outliers
    if length(rttheta)>1
        % check for theta outside sigma from mean
        rtthsig=std(rttheta); 
        rtthbar = mean(rttheta); 
        rtthdiff = abs(rtthbar-rttheta); 
        checkrtth=rtthdiff>rtthsig;
        rtpts1(checkrtth,:)=[]; rtpts2(checkrtth,:)=[];
        rtrho(checkrtth,:)=[];
        % check for rho outside sigma from mean
        rtrhosig=std(rtrho);
        rtrhobar = mean(rtrho);
        rtrhodiff = abs(rtrhobar-rtrho);
        checkrtrho=rtrhodiff>rtrhosig; 
        rtpts1(checkrtrho,:)=[]; rtpts2(checkrtrho,:)=[];
        rtpts=[rtpts1;rtpts2];
    else
        rtpts=[rtpts1;rtpts2];
    end
    % remove left outliers
    if length(lttheta)>1
        % check for theta outside sigma from mean
        ltthsig=std(lttheta);  
        ltthbar = mean(lttheta); 
        ltthdiff = abs(ltthbar-lttheta); 
        checkltth=ltthdiff>ltthsig; 
        ltpts1(checkltth,:)=[]; ltpts2(checkltth,:)=[];
        ltrho(checkltth,:)=[];
        % check for rho outside sigma from mean
        ltrhosig=std(ltrho);
        ltrhobar = mean(ltrho);
        ltrhodiff = abs(ltrhobar-ltrho);
        checkltrho=ltrhodiff>ltrhosig;
        ltpts1(checkltrho,:)=[]; ltpts2(checkltrho,:)=[];
        ltpts=[ltpts1;ltpts2];
    else
        ltpts=[ltpts1;ltpts2];
    end
    
    
    % update the slopes for left and right lines if new points exist
    if ~isempty(rtpts)
        rtp = polyfit(rtpts(:,1),rtpts(:,2),1);
        rtm = rtp(1); rtb = rtp(2);
            
        rtX = [min(rtpts(:,1));max(rtpts(:,1))]; 
    end
    if ~isempty(ltpts)
        ltp = polyfit(ltpts(:,1),ltpts(:,2),1);
        ltm = ltp(1); ltb = ltp(2);
            
        ltX = [min(ltpts(:,1));max(ltpts(:,1))]; 
    end
    
    % inititiate the output vector on the first loop
    if isempty(y1)
        y1=[rtm;rtb;ltm;ltb];
    end
    
    % Kalman filter
    % prediction
    yp=y1; % output prediction
    Pp=P1+Q; % error covariance prediction
    
    % correction
    K=Pp/(Pp+R); % Kalman gain
    y1=yp+K*([rtm;rtb;ltm;ltb]-yp); % updated output
    P1=(eye(4)-K)*Pp; % updated error covariance
    
% uncomment to plot filtered line
    rtY = y1(1).*rtX+y1(2);
    ltY = y1(3).*ltX+y1(4);
    
    % highlight the longest line segment
    plot(rtX,rtY,ltX,ltY,'LineWidth',2,'Color','red');
    