% ����·�����ĺ���
function [path_cost,cost] = calpath_cost(Path, x, mapdata)

cost = 0;


    [~, m] = size(Path);
    for j = 1 : m - 1
        % ��i�����У������ұ��1.2.3...��
        x_now = mod(Path(1, j)-1, x) + 1; 
        % ��i�����У����ϵ��±����1.2.3...��
        y_now = fix((Path(1, j)-1) / x) + 1;
        % ��i+1�����С���
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
path_cost=log(cost); %�����ٻ�