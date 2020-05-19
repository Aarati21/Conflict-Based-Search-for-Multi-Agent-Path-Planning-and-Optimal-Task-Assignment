function [N,G,D,C,S,P,F,M] = readproblem(filename)

% READPROBLEM Read problem definition text file.
%   Params:
%         filename    - name of problem text file
%         N           - Number of Agents (1x1)
%         G           - Number of Goals (1x1)
%         D           - dimensions of map (1x2)
%         C           - collision threshold for map (1x1)
%         S           - robot start location (Nx2)
%         P           - pickup locations (Mx2)
%         F           - delivery locations (Mx2)
%         M           - map cell costs (PxQ where P = D(1), Q = D(2))

FID = fopen(filename, 'r');

if(fgetl(FID) ~= 'N')
    fprintf('Error parsing problem file.')
    return;
end
N = fscanf(FID, '%d')';

if(fgetl(FID) ~= 'G')
    fprintf('Error parsing problem file.')
    return;
end
G = fscanf(FID, '%d')';

if(fgetl(FID) ~= 'D')
    fprintf('Error parsing problem file.')
    return;
end
D = fscanf(FID, '%d,%d')';

if(fgetl(FID) ~= 'C')
    fprintf('Error parsing problem file.')
    return;
end
C = fscanf(FID, '%d');

if(fgetl(FID) ~= 'S')
    fprintf('Error parsing problem file.')
    return;
end
S = textscan(FID, '%f%f', 'CollectOutput', true, 'Delimiter', ',');
S = S{1};

if(fgetl(FID) ~= 'P')
    fprintf('Error parsing problem file.')
    return;
end
P = textscan(FID, '%f%f', 'CollectOutput', true, 'Delimiter', ',');
P = P{1};

if(fgetl(FID) ~= 'F')
    fprintf('Error parsing problem file.')
    return;
end
F = textscan(FID, '%f%f', 'CollectOutput', true, 'Delimiter', ',');
F = F{1};

if(fgetl(FID) ~= 'M')
    fprintf('Error parsing problem file.')
    return;
end
formatSpec = repmat(replace(cat(2,repmat('%f,', 1, D(2)), '\n'), ',\n', '\n'), 1, D(1));
M = reshape(fscanf(FID, formatSpec), D(2), D(1))';

end