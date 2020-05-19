function makemap(n,m,a,prob)

% n - width
% m - height
% a - num of agents
% prob - probability of obstacle

fileID = fopen('newmap.txt','w');

map = zeros(n,m);
collision = 10;

for i = 1:n
    for j = 1:m
        probab = rand;
        if (probab < prob)
            x = 50;
        else
            x = 1;
        end
        map(i,j) = x;
    end
end

k = 1;
x1 = randi([1,n],1,1);
x2 = randi([1,m],1,1);
pairs = [x1,x2];
while (k < 3*a+1)
    x1 = randi([1,n],1,1);
    x2 = randi([1,m],1,1);
    found = 0;
    for ii = 1:k
        if (x1 == pairs(ii,1) && x2 == pairs(ii,2))
            found = 1;
        end
    end
    if (found == 0 && map(x1,x2) < collision)
        pairs = [pairs;[x1,x2]];
        k = k+1;
    end
end

fprintf(fileID,"N\n%d\n",a);
fprintf(fileID,"G\n%d\n",a);
fprintf(fileID,"D\n%d,%d\n",n,m);
fprintf(fileID,"C\n%d\n",10);
fprintf(fileID,"S\n");
for j = 2:a+1
    fprintf(fileID,"%d,%d\n",pairs(j,1),pairs(j,2));
end
fprintf(fileID,"P\n");
for j = a+2:2*a+1
    fprintf(fileID,"%d,%d\n",pairs(j,1),pairs(j,2));
end
fprintf(fileID,"F\n");
for j = 2*a+2:3*a+1
    fprintf(fileID,"%d,%d\n",pairs(j,1),pairs(j,2));
end
fprintf(fileID,"M\n");
for i = 1:n
    for j = 1:m
        if (j ~= m)
            fprintf(fileID,"%d,",map(i,j));
        else
            fprintf(fileID,"%d",map(i,j));
        end
    end
    fprintf(fileID,"\n");
end

fclose(fileID);

end