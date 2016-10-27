function [leg, x_new, y_new] = IKApproach(x, y, nIter, t, mu, l1, l2, l3, z)
%IKAPPROACH Summary of this function goes here
%   Detailed explanation goes here

    legs = zeros(1,18);
    offset = 0.1;
    
    gamma = 5;

    dt = 0.05;

    w_y = 4;

    K = [ 0 -1 -1  1  1 -1;
         -1  0  1 -1 -1  1;
         -1  1  0 -1 -1  1;
          1 -1 -1  0  1 -1;
          1 -1 -1  1  0 -1;
         -1  1  1 -1 -1  0];

    shoulders1 = [1:3:18]; % joint IDs of the shoulders
    shoulders1Corr = [1 -1 1 -1 1 -1]; % correction factor for left/right legs
    shoulderOffsets = [-1 1 0 0 1 -1] * pi/4; % offset so that legs are more spread out
    shoulder2Offsets = [1 1 1 1 1 1] * pi/4;
    shoulders2 = [2:3:18]; % joint IDs of the second shoulder joints
    elbows     = [3:3:18]; % joint IDs of the elbow joints

    limitCycle = sqrt(2); % limit cycle of the CPG (strangely it's not mu, I'll look into that...
%     shoulder1Amp = pi/6 / limitCycle; % amplitude of the shoulder joints (horizontal)
%     shoulder2Amp = pi/5 / limitCycle; % amplitude of the 2nd shoulder joints (vertical)

    if t < nIter
        tic();

        dx = -y(t,:) .* w_y + gamma*(mu.^2 - (x(t,:).^2 + y(t,:).^2)) .* x(t,:);
        dy =  x(t,:) .* w_y + gamma*(mu.^2 - (x(t,:).^2 + y(t,:).^2)) .* y(t,:) + (K*y(t,:)')'/6;
        x(t+1,:) = x(t,:) + dx * dt;
        y(t+1,:) = y(t,:) + dy * dt;
        legs(shoulders1) = x(t+1,:).*shoulders1Corr;
        legs(shoulders2) = max(0,y(t+1,:))+shoulder2Offsets;
        legs(elbows) = -legs(shoulders2);
        
        offsety = 0;%(l1+sqrt(2)*l2/2)/sqrt(2);
        offsetx = 0;%(l1+sqrt(2)*l2/2)-offsety; 
        
        for i = 1 : 6
            if i == 3 || i == 4
                continue;
            end
            theta_1 = legs(3*i-2);
            theta_2 = legs(3*i-1);
            theta_3 = legs(3*i);
           
            %theta_1 = theta_1+shoulderOffsets(i);
            pos_x = l1*cos(theta_1)+(l2*cos(theta_2))*cos(theta_1)
            pos_y = l1*sin(theta_1)+(l2*cos(theta_2))*sin(theta_1)
            legs(3*i-2) = theta_1;
            pos_y = tan(theta_1) * pos_x;
            %pos_x = pos_x - offsetx
            %pos_y = pos_y + offsety
            
            
            p1_x = l1*cos(theta_1);
            p1_y = l1*sin(theta_1);
            d = sqrt((pos_x-p1_x)*(pos_x-p1_x)+(pos_y-p1_y)*(pos_y-p1_y) + z*z);
            theta_3 = acos((l3*l3+l2*l2-d*d)/(2*l2*l3));
            alpha = acos((l2*l2+d*d-l3*l3)/(2*l2*d));
            theta_2 = alpha-pi/2+acos(z/d);
            legs(3*i-1) = theta_2;
            legs(3*i) = theta_3-pi+1.05;
            theta_1 = legs(3*i-2);
            theta_2 = legs(3*i-1);
            theta_3 = legs(3*i)+pi-1.05;
            ik_x = l1*cos(theta_1)+(l3*cos(theta_3)+l2*cos(theta_2))*cos(theta_1)
            ik_y = l1*sin(theta_1)+(l3*cos(theta_3)+l2*cos(theta_2))*sin(theta_1)
            
        end
%         legs(shoulders1) = (shoulderOffsets + x(t+1,:)) .* shoulders1Corr;
%         legs(shoulders2) = (max(0,y(t+1,:)) + shoulder2Offsets);
           
        loopTime = toc();
        pause(max(0, dt - loopTime));
        leg = legs;
        x_new = x;
        y_new = y;
    end
end

