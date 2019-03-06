function [rtm,rtb,ltm,ltb,y1,P1,rtX,ltX]=cp002(pic,Q,R,shape,y1,P1,rtm,rtb,ltm,ltb,rtX,ltX)

%AFTERNOON LIGHTING

% parameters for image masking
    target_x=0.1; target_y=0.2*shape(1); oset=0.9;
    a=[0,shape(2)*target_x,shape(2)*(1-target_x),shape(2),shape(2),0];% x coordinates of trapezoid
    b=[shape(1)*oset,target_y,target_y,shape(1)*oset,shape(1),shape(1)]; % y coordinates of trepezoid
    
    gray_pic = rgb2gray(pic); % gray scale
    edge_pic = edge(gray_pic,'canny',[0.1 0.6]); % find edges
    % canny thresholds: sample video=[0.1 0.45]; midday on veh=[0.1 0.6]
    
    bw=roipoly(pic,a,b); % build masking area
    BW=(edge_pic(:,:,1)&bw); % combine masking with edge pic
    
% uncomment to plot picture
%     figure(1)
%     hold off
%     imshow(BW); hold on
%     
    [H,theta,rho]=hough(BW,'Theta',-45:45);
    
    P = houghpeaks(H,4,'Threshold',0.3*max(max(H)));

    lines = houghlines(BW,theta,rho,P,'FillGap',10,'MinLength',7);
    
    rtpts1=[];rtpts2=[]; ltpts1=[];ltpts2=[];
    lttheta=[]; rttheta=[]; ltrho=[]; rtrho=[];
    for k = 1:length(lines)

% Uncomment to plot all lines found
%         xy = [lines(k).point1; lines(k).point2];
%         plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');
        
        % determine if the line is the left or right lane bound
        if lines(k).theta<0 %&& lines(k).theta>-60
            rtpts1=[rtpts1;lines(k).point1];
            rtpts2=[rtpts2;lines(k).point2];
            rttheta=[rttheta;lines(k).theta];
            rtrho=[rtrho;lines(k).rho];
        elseif lines(k).theta>=0 %&& lines(k).theta<60
            ltpts1=[ltpts1;lines(k).point1];
            ltpts2=[ltpts2;lines(k).point2];
            lttheta=[lttheta;lines(k).theta];
            ltrho=[ltrho;lines(k).rho];
        end
    end
    
    if length(rttheta)>1
        % check for theta outside 2 sigma
        rtthsig=std(rttheta); 
        rtthbar = mean(rttheta); 
        rtthdiff = abs(rtthbar-rttheta); 
        checkrtth=rtthdiff>rtthsig;
        rtpts1(checkrtth,:)=[]; rtpts2(checkrtth,:)=[];
        rtrho(checkrtth,:)=[];
        % check for rho outside 2 sigma
        rtrhosig=std(rtrho);
        rtrhobar = mean(rtrho);
        rtrhodiff = abs(rtrhobar-rtrho);
        checkrtrho=rtrhodiff>rtrhosig; 
        rtpts1(checkrtrho,:)=[]; rtpts2(checkrtrho,:)=[];
        rtpts=[rtpts1;rtpts2];
    else
        rtpts=[rtpts1;rtpts2];
    end
    if length(lttheta)>1
        % check for theta outside 2 sigma
        ltthsig=std(lttheta);  
        ltthbar = mean(lttheta); 
        ltthdiff = abs(ltthbar-lttheta); 
        checkltth=ltthdiff>ltthsig; 
        ltpts1(checkltth,:)=[]; ltpts2(checkltth,:)=[];
        ltrho(checkltth,:)=[];
        % check for rho outside 2 sigma
        ltrhosig=std(ltrho);
        ltrhobar = mean(ltrho);
        ltrhodiff = abs(ltrhobar-ltrho);
        checkltrho=ltrhodiff>ltrhosig;
        ltpts1(checkltrho,:)=[]; ltpts2(checkltrho,:)=[];
        ltpts=[ltpts1;ltpts2];
    else
        ltpts=[ltpts1;ltpts2];
    end
    
%     disp(rttheta)
%     disp(lttheta)
    
    % update the slopes for left and right lines if new lines exist
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
    
%     if isempty(rtm) && ~isempty(ltm)
%         rtm=-ltm;
%         rtX = shape(2)-ltX;
%     elseif ~isempty(rtm) && isempty(ltm)
%         ltm=-rtm;
%         ltX = shape(2)-rtX;
%     elseif ~isempty(rtm) && ~isempty(ltm)
%         o=0.3;
%         rtm=-999;rtb=-rtm*(1-o)*shape(2);
%         ltm=-999;ltb=-ltm*(1-o)*shape(2);
%         rtX=[(1-o)*shape(2) (shape(1)-rtb)/rtm]; ltX=[o*shape(2) (shape(1)-ltb)/ltm];
%     end
    
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
%     plot(rtX,rtY,ltX,ltY,'LineWidth',2,'Color','red');
    