% ����·��ˤ���ʺ���
function [path_risk,risk] = calpath_risk(Path, x, maprisk)

risk = 1;

pathsafety=1;
[~, m] = size(Path);
for j = 1 : m - 1
        % ��i�����У������ұ��1.2.3...��
        x_now = mod(Path(1, j)-1, x) + 1; 
        % ��i�����У����ϵ��±����1.2.3...��
        y_now = fix((Path(1, j)-1) / x) + 1;
        % ��i+1�����С���
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

% path_risk=(risk*100)-(90*ones(1, n));  %ˤ������������Ϊ0.99...�������ֶ�̫С�����Գ�100-99���õ��ٷ�λ�Ժ��С������
% path_risk=mapminmax(path_risk,1,limit);
% path_risk=mapminmax(path_risk,1,10);
path_risk=log(((risk*100)-35)*10); %ˤ������������Ϊ0.99...�������ֶ�̫С�����Գ�100-99���õ��ٷ�λ�Ժ��С�����ݣ��ٳ�10ȡln�������ٻ�
