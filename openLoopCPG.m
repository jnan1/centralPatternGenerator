function [leg, x_new, y_new] = openLoopCPG(x, y, nIter, t, mu)
    legs = zeros(1,18);

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
    shoulderOffsets = [-1 -1 0 0 1 1] * pi/4; % offset so that legs are more spread out
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

        legs(shoulders1) = (shoulderOffsets + x(t+1,:)) .* shoulders1Corr;
        legs(shoulders2) = (max(0,y(t+1,:)) + shoulder2Offsets);
        legs(elbows) = -legs(shoulders2);

        loopTime = toc();
        pause(max(0, dt - loopTime));
        leg = legs;
        x_new = x;
        y_new = y;
    end
end

