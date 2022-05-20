% 计算路径功耗函数
function [path_cost,cost] = calpath_cost(Path, x, mapdata)

cost = 0;


    [~, m] = size(Path);
    for j = 1 : m - 1
        % 点i所在列（从左到右编号1.2.3...）
        x_now = mod(Path(1, j)-1, x) + 1; 
        % 点i所在行（从上到下编号行1.2.3...）
        y_now = fix((Path(1, j)-1) / x) + 1;
        % 点i+1所在列、行
        x_next = mod(Path(1, j)-1, x) + 1;
        y_next = fix((Path(1, j)-1)/ x) + 1;
        if abs(x_now - x_next) + abs(y_now - y_next) == 1
            if mapdata(x_now,y_now)~=inf
                cost = cost + 1*mapdata(x_now,y_now);
            else
                cost=inf;
            end
        else
            if mapdata(x_now,y_now)~=inf
                cost = cost + sqrt(2)*mapdata(x_now,y_now) ;
            else
                cost=inf;
            end
        end
    end

% path_cost=mapminmax(cost,1,limit);
% path_cost=mapminmax(cost,1,10);
path_cost=log(cost); %无量纲化