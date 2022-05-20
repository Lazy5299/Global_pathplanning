% ����·������ʱ�亯��
function [path_time,time] = calpath_time(Path, x, mapspeed)

time = 0;

[~, m] = size(Path);
for j = 1 : m - 1
        % ��i�����У������ұ��1.2.3...��
        x_now = mod(Path(1, j)-1, x) + 1; 
        % ��i�����У����ϵ��±����1.2.3...��
        y_now = fix((Path(1, j)-1) / x) + 1;
        % ��i+1�����С���
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
path_time=log(time);%�����ٻ�