% 计算路径摔倒率函数
function [path_risk,risk] = calpath_risk(Path, x, maprisk)

risk = 1;

pathsafety=1;
[~, m] = size(Path);
for j = 1 : m - 1
        % 点i所在列（从左到右编号1.2.3...）
        x_now = mod(Path(1, j)-1, x) + 1; 
        % 点i所在行（从上到下编号行1.2.3...）
        y_now = fix((Path(1, j)-1) / x) + 1;
        % 点i+1所在列、行
        x_next = mod(Path(1, j + 1)-1, x) + 1;
        y_next = fix((Path(1, j + 1)-1) / x) + 1;
        if abs(x_now - x_next) + abs(y_now - y_next) == 1
            if maprisk(x_now,y_now)~=inf
                pathsafety =pathsafety*(1-maprisk(x_now,y_now));
            else
                pathsafety=inf;
            end
        else
            if maprisk(x_now,y_now)~=inf
                pathsafety =pathsafety*(1-sqrt(2)*maprisk(x_now,y_now));
            else
                pathsafety=inf;
            end
        end
    if pathsafety==inf
        risk=inf;  
    else
        risk=(1-pathsafety); 
    end
end

% path_risk=(risk*100)-(90*ones(1, n));  %摔倒率最后基本都为0.99...导致区分度太小，所以乘100-99，得到百分位以后的小数数据
% path_risk=mapminmax(path_risk,1,limit);
% path_risk=mapminmax(path_risk,1,10);
path_risk=log(((risk*100)-35)*10); %摔倒率最后基本都为0.99...导致区分度太小，所以乘100-99，得到百分位以后的小数数据，再乘10取ln，无量纲化
