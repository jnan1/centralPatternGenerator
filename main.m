
legs = zeros(1,18);

sweep = pi/12; % step sweep
a = .3; % semi major-axis of the limit-ellipse
b = 1.; % semi minor-axis of the limit-ellipse
gamma = 20;

T = 20;
dt = 0.005;
nIter = round(T/dt);

w = zeros(1,6);
x = [.1 -.1 -.1 .1 .1 -.1; zeros(nIter,6)];
y = zeros(nIter+1, 6);

theta1 = zeros(nIter,6);
theta2 = zeros(nIter,6);
theta3 = zeros(nIter,6);

K = [ 0 -1 -1  1  1 -1;
     -1  0  1 -1 -1  1;
     -1  1  0 -1 -1  1;
      1 -1 -1  0  1 -1;
      1 -1 -1  1  0 -1;
     -1  1  1 -1 -1  0];

shoulders1          = 1:3:18; % joint IDs of the shoulders
shoulders1Corr      = [1 -1 1 -1 1 -1]; % correction factor for left/right legs
shoulder1Offsets    = [-1 -1 0 0 1 1] * pi/4.5; % offset so that legs are more spread out
shoulders2          = 2:3:18; % joint IDs of the second shoulder joints
elbows              = 3:3:18; % joint IDs of the elbow joints
shoulder2Offsets    = pi/6 * ones(1,6);
stepHeight = 0.1;
bodyHeight = 0.15;

pause(1);
t = 1;
legs(shoulders1) = (shoulder1Offsets + x(1,:)) .* shoulders1Corr;
legs(shoulders2) = shoulder2Offsets + max(0,y(1,:));
legs(elbows) = -legs(shoulders2);

L1 = 0.12; L2 = 0.19; Hfoot = 0.1849; xKy = 0.07; moduleLen = .097;
radCentral = 2*moduleLen + .063-.0122 + L1*cos(shoulder2Offsets(1));
d = 2 * tan(sweep/b)*radCentral;
r = moduleLen + xKy ./ cos(shoulder1Offsets) + moduleLen*sin(abs(shoulder1Offsets));
r(3:4) = radCentral;
mu = acos( sqrt( (4*r.^2 + sqrt(2)*sqrt(r.^2 .* (d.^2 + 8*r.^2 - d.^2 * cos(4*shoulder1Offsets))) + 2*d.^2 * sin(shoulder1Offsets).^2) ./ (d.^2 + 4*r.^2) ) / sqrt(2) );
plt = SnakeMonsterPlotter();

theta1 = zeros(nIter,6);
theta2 = zeros(nIter,6);
theta3 = zeros(nIter,6);

K = [ 0 -1 -1  1  1 -1;
     -1  0  1 -1 -1  1;
     -1  1  0 -1 -1  1;
      1 -1 -1  0  1 -1;
      1 -1 -1  1  0 -1;
     -1  1  1 -1 -1  0];


L1 = 0.12; L2 = 0.19; Hfoot = 0.1849; xKy = 0.07; moduleLen = .097;

for t = 1 : nIter
    tic();
    dx = -a*y(t,:) .* w + gamma.*(mu.^2 - (b.*x(t,:).^2 + a.*y(t,:).^2)) .* b.*x(t,:);
    dy =  b*x(t,:) .* w + gamma.*(mu.^2 - (b.*x(t,:).^2 + a.*y(t,:).^2)) .* a.*y(t,:) + (K*y(t,:)')'./6;
    x(t+1,:) = x(t,:) + dx * dt;
    y(t+1,:) = y(t,:) + dy * dt;
    
    legs(shoulders1) = (shoulder1Offsets + x(t+1,:)) .* shoulders1Corr
    
    % Ky's Magic
    z = stepHeight * max(0,a.*y(t+1,:)) ./ mu - bodyHeight;
    r = xKy ./ cos(legs(shoulders1)) + moduleLen*sin(abs(legs(shoulders1)));
    r2 = sqrt(r.^2 + z.^2);
    legs(shoulders2) = acos( (-L2^2 + L1^2 + r2.^2) ./ (2 .* r2 .* L1) ) + sign(z) .* acos(r ./ r2)
    legs(elbows) = acos( (L1^2 + L2^2 - r2.^2) ./ (2 .* L1 .* L2)) - pi/2 - 0.3707 % extra angle created by the bend
    
    theta1(t,:) = legs(shoulders1);
    theta2(t,:) = legs(shoulders2);
    theta3(t,:) = legs(elbows);
    
    plt.plot(legs');
    loopTime = toc();
    pause(max(0, dt - loopTime));
end
% %global mu
% disp('finish your input with h');
% disp('clear input up with c');
% mu = pi/6;
% nIter = round(50/0.05);
% x = [.1 -.1 .1 -.1 .1 -.1; zeros(nIter,6)];
% y = zeros(nIter+1, 6);
% plt= SnakeMonsterPlotter();
% fig_h = plt.getFigure();
% set(fig_h,'KeyPressFcn',@callbackFunction);
% 
% for t = 1 : nIter
%    [legs, x_new, y_new] = IKApproach(x, y, nIter, t, mu, 0.098, 0.128, 0.2, 0.2);
% %    [legs, x_new, y_new] = openLoopCPG(x, y, nIter, t, mu);
%     plt.plot(legs');
%     x = x_new;
%     y = y_new;
% end