% call generate_points first
numV = length(x);
fileID = fopen('edges.txt','r');
edgeList = fscanf(fileID,'%d %d',[2 Inf]);
fclose(fileID);
edgeList = edgeList';
edgeList = edgeList + 1;
m = sparse(edgeList(:,1), edgeList(:,2), round(sqrt( (x(edgeList(:,1)) - x(edgeList(:,2))).^2 + (y(edgeList(:,1)) - y(edgeList(:,2))).^2 )), numV, numV);
%fprintf('%d\n%d\n%d\n0\n\n', [r,y,x]');

fileID = fopen('graph.txt','w');
for i = 0:length(x)-1
    l = m(i+1,:);
    fprintf(fileID,'%d\n%d\n%d\n', [i,y(i+1),x(i+1)]');
    fprintf(fileID,'%d\n', nnz(l));
    indices = (find(l)-1)';
    values = nonzeros(l);
    fprintf(fileID,'%d\nN\n%d\n', [indices, values]');
    fprintf(fileID,'\n');
end
fclose(fileID);
