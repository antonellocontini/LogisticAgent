% provide a points.png file, where red pixels are vertices, the rest
% should be black
I = imread("points.png");
Ir = flip(I,1);
[x,y] = find(Ir==255);
r = (0:length(x)-1)';
fprintf('%d\n%d\n%d\n0\n\n', [r,y,x]');