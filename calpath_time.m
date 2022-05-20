% 计算路径消耗时间函数
function [path_time,time] = calpath_time(Path, x, mapspeed)

time = 0;

[~, m] = size(Path);
for j = 1 : m - 1
        % 点i所在列（从左到右编号1.2.3...）
        x_now = mod(Path(1, j)-1, x) + 1; 
        % 点i所在行（从上到下编号行1.2.3...）
        y_now = fix((Path(1, j)-1) / x) + 1;
        % 点i+1所在列、行
        x_next = mod((Path(1, j + 1)-1), x) + 1;
        y_next = fix((Path(1, j + 1)-1) / x) + 1;
        if abs(x_now - x_next) + abs(y_now - y_next) == 1
            if mapspeed(x_now,y_now)~=inf
                time = time + 1*mapspeed(x_now,y_now);
            else
                time=inf;
                break
            end
        else
            if mapspeed(x_now,y_now)~=inf
                time = time + sqrt(2)*mapspeed(x_now,y_now) ;
            else
                time=inf;
                break
            end
        end
end

% path_time=mapminmax(time,1,limit);
% path_time=mapminmax(time,1,10);
path_time=log(time);%无量纲化