function DFS_Maze(maze,mapdata,mapspeed,maprisk,secost,serisk,sespeed,A)

% 本函数用深度优先遍历(回溯法)来求解迷宫的所有路径
% maze:是迷宫矩阵，其中0表示可以去走的路
% 1表示障碍
% 2表示入口
% 3表示出口
% 5表示路径




if nargin == 0
maze = G;
end
% % 定义四个方向
% directions = kron(eye(2),[-1,1]);
% 定义八个方向
directions = [-1 1 0 0 1 -1 1 -1;0 0 -1 1 1 -1 -1 1];
% 路径个数
pathnumber = 0;
sol = 0;
j=1;
Cost=[]; %消耗
Speed=[]; %长度
Fallrate=[]; %摔倒率
ALLMAP = {} ;
[I,J] = find(maze == 2);% 找到起点
search(I,J);
COST=Cost';
SPEED=Speed';
FALLRATE=Fallrate';
pathnumber=['有效可行路线数量：',num2str(sol)];
costdata=['各路线功耗：',num2str(Cost)];
speeddata=['消耗时间：',num2str(Speed)];
fallrate=['各路线平均摔倒率：',num2str(Fallrate)];
disp(pathnumber);
disp(costdata);
disp(speeddata);
disp(fallrate);              %显示数据
xlswrite('x功耗.xlsx',COST);
xlswrite('y消耗时间.xlsx',SPEED);
xlswrite('z摔倒率.xlsx',FALLRATE); %写入表格


% 该函数判断该点是否可以经过
function z = cango(x,y)
% 用try来判断边界
z = true;
try
if ismember(maze(x,y),[1,2,5])% 路障或者已经走过
z = false;
end
catch
z = false; % 边界
end
end


function search(x,y)
routeindexes=find(maze==5);  %路线
BN=size(routeindexes,1)+2;    %路线长度，加上起点终点
if BN < A*4.5  %限制路线格数
    if maze(x,y) == 3    % 找到出口
        disp(maze); % 打印路径
        pathnumber = pathnumber + 1;
        filename=strcat('Path_',num2str(pathnumber),'.mat');
        save(['C:\Users\lazy\Desktop\10.22 19基础上，查找全部路线可限制路径格数\allpath\',filename],'maze');    %保存路径
        sol = sol + 1;% 解的个数增加
        costindexes=mapdata(routeindexes);  %路线中每个栅格的功耗
        speedindexes=mapspeed(routeindexes);   %路线中每个栅格的距离
        fallrateindexes=maprisk(routeindexes); %路线中每个栅格的摔倒率
        c=sum(costindexes)+2*secost;               %路线能量
        l=sum(speedindexes)+2*sespeed;               %路线长度
        f=1;
        for no=1:length(fallrateindexes)
            f=f*(1-fallrateindexes(no));               
        end
        fe=1-f*(1-serisk)*(1-serisk);                            %路线摔倒率
        Cost(j)=c;
        Speed(j)=l;
        Fallrate(j)=fe;
        ALLMAP{j}=maze;
        j=j+1;
        return % 返回
    end
end



% 搜索8个方向
for i = 1 : 8
    if cango(x + directions(1,i),y + directions(2,i)) % 如果可以走
        if maze(x + directions(1,i),y + directions(2,i)) ~= 3
            maze(x + directions(1,i),y + directions(2,i)) = 5;% 记录下来
        end
        search(x + directions(1,i),y + directions(2,i)); % 继续寻找下一个点
        if maze(x + directions(1,i),y + directions(2,i)) ~= 3
            maze(x + directions(1,i),y + directions(2,i)) = 0; % 回到该节点，继续寻找下一个方向
        end
    end
end
end
end