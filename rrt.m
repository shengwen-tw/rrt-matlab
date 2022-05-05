rng(100,'twister'); % for repeatable result

%==========%
% read map %
%==========%
image = imread('./map1.png');
grayimage = rgb2gray(image);
bwimage = grayimage < 0.5;
map = binaryOccupancyMap(bwimage);
show(map)
hold on;

%===============%
% RRT algorithm %
%===============%

%parameters
MAX_ITERATION=1e5;
p_start = [100, 100];
p_goal = [1500, 700];
goal_threshold = 100;
delta = 50;

map_size = [map.XWorldLimits(2); map.YWorldLimits(2)];
tree.vertex(1).p = p_start;
tree.vertex(1).p_last = p_start;
tree.vertex(1).dist = 0;
tree.vertex(1).index = 0;
vertex_cnt = 1;

figure(1);
plot(p_start(1), p_start(2), 'bo', 'MarkerSize', 5, 'MarkerFaceColor', 'r');
hold on;
plot(p_goal(1), p_goal(2), 'bo', 'MarkerSize', 5, 'MarkerFaceColor', 'g');
hold on;

for iteration = 1:MAX_ITERATION
    %generate random point
    p_rand = round([rand()*map_size(1); rand()*map_size(2)]);
    
    %find the nearest point to the random point
    min_dist = 1e10;
    nearest_index = 0;
    for i = 1:vertex_cnt
        p = tree.vertex(i).p;
        dist = sqrt((p(1)-p_rand(1))^2 + (p(2)-p_rand(2))^2);
        if dist < min_dist
            min_dist = dist;
            nearest_index = i;
        end
    end
    
    %get the nearest point
    p_near = tree.vertex(nearest_index).p;
    
    %generate a new RRT point
    dir = (p_rand - p_near) / min_dist;
    p_new = p_near + (dir * delta);
    
    %check collision free
    if check_collision(p_near, p_new, map) == false
        continue;
    end
    
    %add the new point into the tree
    vertex_cnt = vertex_cnt + 1;
    tree.vertex(vertex_cnt).p = p_new;
    tree.vertex(vertex_cnt).p_last = p_near;
    tree.vertex(vertex_cnt).dist = min_dist;
    
    %plot
    plot(p_new(1), p_new(2), 'bo', 'MarkerSize', 2);
    line([p_new(1) p_near(1)], [p_new(2) p_near(2)]);
    hold on;
    
    %check if the new point is close to the the goal
    dist_to_goal = sqrt((p_new(1)-p_goal(1))^2 + (p_new(2)-p_goal(2))^2);
    if dist_to_goal <= goal_threshold
        line([p_new(1) p_goal(1)], [p_new(2) p_goal(2)]);
        hold on;
        break;
    end
    
    pause(0.01)
end

%===============================%
% trace the trajectory solution %
%===============================%
path_cnt = 1;
rtt.path(1).point = p_goal;
%
path_cnt = path_cnt + 1;
rtt.path(2).point = p_new;
%
node_index = retrieve_node_index(tree, p_new);
path_cnt = path_cnt + 1;
rtt.path(path_cnt).point = tree.vertex(node_index).p_last;
%
while ~(rtt.path(path_cnt).point(1) == p_start(1) && ...
        rtt.path(path_cnt).point(2) == p_start(2))
    node_index = retrieve_node_index(tree, rtt.path(path_cnt).point);
    path_cnt = path_cnt + 1;
    rtt.path(path_cnt).point = tree.vertex(node_index).p_last;
end

for i = (path_cnt-1):-1:1
    p_now = rtt.path(i).point;
    p_next = rtt.path(i+1).point;
    line([p_now(1) p_next(1)], [p_now(2) p_next(2)], 'color', 'r', 'LineWidth',2);
    hold on;
    pause(0.01);
end

disp(iteration)

pause;
close all;

function feasible=check_collision(p_start, p_end, map)
    feasible = true;
    dist = sqrt((p_end(1)-p_start(1))^2 + (p_end(2)-p_start(2))^2);

    dir = (p_end - p_start) / dist;
    for dist_inc = 0:0.5:dist
        p_test = p_start + dist_inc * dir;
        
        p_test_1 = ceil(p_test);
        p_test_2 = floor(p_test);
        p_test_3 = [ceil(p_test(1)) floor(p_test(2))];
        p_test_4 = [floor(p_test(1)) ceil(p_test(2))];
        
        if ~(check_feasible(p_test_1, map) && check_feasible(p_test_2, map) && ...
             check_feasible(p_test_3, map) && check_feasible(p_test_4, map))
            feasible = false;
            break;
        end
    end
end

function feasible=check_feasible(point, map)
    feasible = true;
    world_edge_x = map.XWorldLimits(2);
    world_edge_y = map.YWorldLimits(2);
    
    %index of occupancy map starts from 1
    if ~(point(1) >= 1 && point(1) <= world_edge_x && ...
         point(2) >= 1 && point(2) <= world_edge_y && ...
         getOccupancy(map, [point(1), point(2)]) == 0)
        feasible = false;
    end
end

function index=retrieve_node_index(tree, point)
    index = 1;
    while tree.vertex(index).p(1) ~= point(1) || ...
          tree.vertex(index).p(2) ~= point(2)
      index = index + 1;
    end
end