%lab1

R = zeros(4,4);
R01 = zeros(4,4);
R12 = zeros(4,4);
R23 = zeros(4,4);
R34 = zeros(4,4);
R45 = zeros(4,4);
thetas = zeros(1,5);
alphas = zeros(1,5);
ds = zeros(1,5);
as = zeros(1,5);

for n = 1:1:5
	R(1,1) = cos(thetas(n));
	R(1,2) = -1*sin(thetas(n));
	R(1,3) = 0;
	R(1,4) = as(n-1);
	R(2,1) = sin(thetas(n))*cos(alphas(n-1));
	R(2,2) = cos(thetas(n))*cos(alphas(n-1));
	R(2,3) = -1*sin(alphas(n-1));
	R(2,4) = -1*ds(n)*sin(alphas(n-1));
	R(3,1) = sin(thetas(n))*sin(alphas(n-1));
	R(3,2) = cos(thetas(n))*sin(alphas(n-1));
	R(3,3) = cos(alphas(n-1));
	R(3,4) = ds(n)*cos(alphas(n-1));
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
