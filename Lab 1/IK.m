xd = [-9,6,12,1]'; %where we want our end effector to be
x0 = [6,9,12,1]'; %initial operational state
q = [0,90,0,0]'; %initial joint state
N = 4;

% we will be computing the difference between our guess x0 and end
% destination xd using the Jacobian to update the change in the joint space
% The loop will continue until the x0 is very close to xd
while norm(xd - x0) > 0.01
delx = xd - x0;
difx = delx/N;
J = [-9*sind(q(1))*cosd(q(2))*sind(q(3)) - 9*cosd(q(1))*cosd(q(3))- 12*sind(q(1))*cosd(q(2))-6*sind(q(1))*sind(q(2)), -9*cosd(q(1))*sind(q(2))*sind(q(3)) - 12*cosd(q(1))*sind(q(2)) + 6*cosd(q(1))*cosd(q(2)), 9*cosd(q(1))*cosd(q(2))*cosd(q(3)) + 9*sind(q(1))*sind(q(3)),0;
     9*cosd(q(1))*cosd(q(2))*sind(q(3)) - 9*sind(q(1))*cosd(q(3)) + 12*cosd(q(1))*cosd(q(2)) + 6*cosd(q(1))*sind(q(2)), -9*sind(q(1))*sind(q(2))*sind(q(3)) - 12*sind(q(1))*sind(q(2)) + 6*sind(q(1))*cosd(q(2)), 9*sind(q(1))*cosd(q(2))*cosd(q(3)) - 9*cosd(q(1))*sind(q(3)),0;
     0, 9*cosd(q(2))*sind(q(3)) + 12*cosd(q(2)) + 6*sind(q(2)), 9*sind(q(2))*cosd(q(3)),0;
     0 , 0 , 0 , 0];
dq = pinv(J)*difx; 
q = q + dq;
x0 = x0 + difx;
end

sprintf('Desired location of end effector')
xd'
sprintf('Calculated joint parameters')
q'