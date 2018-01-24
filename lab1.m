%lab1

R = zeros(4,4);
L2 = 12;
L3 = 6;
L4 = 9;
R01 = zeros(4,4);
R12 = zeros(4,4);
R23 = zeros(4,4);
R34 = zeros(4,4);
R45 = zeros(4,4);
alphas = [0,90,90,90,0];
ds = [0,0,L3,0,L4];
as = [0,0,L2,0,0];

for n = 1:1:5
	R(1,1) = cosd(thetas(n));
	R(1,2) = -1*sind(thetas(n));
	R(1,3) = 0;
	R(1,4) = as(n);
	R(2,1) = sind(thetas(n))*cosd(alphas(n));
	R(2,2) = cosd(thetas(n))*cosd(alphas(n));
	R(2,3) = -1*sind(alphas(n));
	R(2,4) = -1*ds(n)*sind(alphas(n));
	R(3,1) = sind(thetas(n))*sind(alphas(n));
	R(3,2) = cosd(thetas(n))*sind(alphas(n));
	R(3,3) = cosd(alphas(n));
	R(3,4) = ds(n)*cosd(alphas(n));
	R(4,1) = 0;
	R(4,2) = 0;
	R(4,3) = 0;
	R(4,4) = 1;

	if n == 1
		R01 = R;
	elseif n == 2
		R12 = R;
	elseif n == 3
		R23 = R;
	elseif n == 4
		R34 = R;
	elseif n == 5
		R45 = R;
    end
end

res = R01*R12*R23*R34*R45
