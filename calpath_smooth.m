% ����·��ƽ���Ⱥ�����ƽ���Ⱥ����в�û�о���ĽǶ���Ϣ��ֻ�гͷ���Ϣ
function [path_smooth,totalangle] = calpath_smooth(Path, x)

pathsmooth = 1;
totalangle = 0;


    [~, m] = size(Path);
    for j = 1 : m - 2
        % ��i�����У������ұ��1.2.3...��
        x_now = mod(Path(1, j)-1, x) + 1; 
        % ��i�����У����ϵ��±����1.2.3...��
        y_now = fix((Path(1, j)-1) / x) + 1;
        % ��i+1�����С���
        x_next1 = mod(Path(1, j + 1)-1, x) + 1;
        y_next1 = fix((Path(1, j + 1)-1) / x) + 1;
        % ��i+2�����С���
        x_next2 = mod(Path(1, j + 2)-1, x) + 1;
        y_next2 = fix((Path(1, j + 2)-1) / x) + 1;
        % cos A=(b?+c?-a?)/2bc
        b2 = (x_now - x_next1)^2 + (y_now - y_next1)^2;
        c2 = (x_next2 - x_next1)^2 + (y_next2 - y_next1)^2;
        a2 = (x_now - x_next2)^2 + (y_now - y_next2)^2;
        cosa = (c2 + b2 - a2) / (2 * sqrt(b2) *  sqrt(c2));
        angle = acosd(cosa);
        radangle=deg2rad(angle);
        
        %����ÿ��·���ܵ��۽Ǵ�С
        totalangle = totalangle+ (pi-radangle);
        
        %�ԽǶȽϴ��ʩ�ӳͷ�
        if angle < 170 && angle > 91     
            pathsmooth = pathsmooth+ 10; %�۽Ǽӽ�С�ͷ�
        elseif angle <= 91 && angle > 46
            pathsmooth = pathsmooth+ 30;  %�е����
        elseif angle <= 46 && angle>=0
            pathsmooth = pathsmooth + 90;   %С��ǼӴ�ͷ�
        else
            pathsmooth = pathsmooth + 5;
        end
    end

% path_smooth=mapminmax(pathsmooth,1,limit);
% path_smooth=mapminmax(pathsmooth,1,10);
path_smooth=log(pathsmooth);%�����ٻ�
