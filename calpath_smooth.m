% 计算路径平滑度函数，平滑度函数中并没有具体的角度信息，只有惩罚信息
function [path_smooth,totalangle] = calpath_smooth(Path, x)

pathsmooth = 1;
totalangle = 0;


    [~, m] = size(Path);
    for j = 1 : m - 2
        % 点i所在列（从左到右编号1.2.3...）
        x_now = mod(Path(1, j)-1, x) + 1; 
        % 点i所在行（从上到下编号行1.2.3...）
        y_now = fix((Path(1, j)-1) / x) + 1;
        % 点i+1所在列、行
        x_next1 = mod(Path(1, j + 1)-1, x) + 1;
        y_next1 = fix((Path(1, j + 1)-1) / x) + 1;
        % 点i+2所在列、行
        x_next2 = mod(Path(1, j + 2)-1, x) + 1;
        y_next2 = fix((Path(1, j + 2)-1) / x) + 1;
        % cos A=(b?+c?-a?)/2bc
        b2 = (x_now - x_next1)^2 + (y_now - y_next1)^2;
        c2 = (x_next2 - x_next1)^2 + (y_next2 - y_next1)^2;
        a2 = (x_now - x_next2)^2 + (y_now - y_next2)^2;
        cosa = (c2 + b2 - a2) / (2 * sqrt(b2) *  sqrt(c2));
        angle = acosd(cosa);
        radangle=deg2rad(angle);
        
        %计算每个路线总的折角大小
        totalangle = totalangle+ (pi-radangle);
        
        %对角度较大的施加惩罚
        if angle < 170 && angle > 91     
            pathsmooth = pathsmooth+ 10; %钝角加较小惩罚
        elseif angle <= 91 && angle > 46
            pathsmooth = pathsmooth+ 30;  %中等锐角
        elseif angle <= 46 && angle>=0
            pathsmooth = pathsmooth + 90;   %小锐角加大惩罚
        else
            pathsmooth = pathsmooth + 5;
        end
    end

% path_smooth=mapminmax(pathsmooth,1,limit);
% path_smooth=mapminmax(pathsmooth,1,10);
path_smooth=log(pathsmooth);%无量纲化
