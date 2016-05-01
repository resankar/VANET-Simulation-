clear;
%axis ([0 500 0 500]);
hold on
% Variable Declarations for the program
Communication_range = 100;
length_of_the_road=5000;
lane1pos = 500;
lane2pos = 503;
lane3pos = 506;
lane4pos = 509;
total_connected_vehicles = 0;
nodes_1_intersect = 0;
nodes_2_intersect = 0;
nodes_3_intersect = 0;
nodes_4_intersect = 0;
count = 0;
counter = 0;
avg_nodes = 0;
time_dur_3_neighbors = 0;
same_comm_neighbors = 0;
avg_V2V = 0;
avg_3_neighbors = 0;
avg_same_comm_neighbors = 0;
v2vcommunication_distance=50;
X=0:1:v2vcommunication_distance;
limit=1;
increment=1;
trafficdensity=[50 100 150 200 250 300 350 400 450 500];

% Loop to initate the process and simulate the model for all the cars in the lane 

for No_of_cars_in_a_lane=100:50:500
    % Defining the vector to store the position of the cars 
    ypos_in_1st_lane=sort(unidrnd(length_of_the_road,[No_of_cars_in_a_lane,1]),1); 
    ypos_in_2nd_lane=sort(unidrnd(length_of_the_road,[No_of_cars_in_a_lane,1]),1);
    ypos_in_3rd_lane=sort(unidrnd(length_of_the_road,[No_of_cars_in_a_lane,1]),1);
    ypos_in_4th_lane=sort(unidrnd(length_of_the_road,[No_of_cars_in_a_lane,1]),1);
    
    % Defining the vector to store the speed of the cars
    speed_for_cars_in_lane1=0.44704.*randi([50 70],No_of_cars_in_a_lane,1);
    speed_for_cars_in_lane2=0.44704.*randi([50 70],No_of_cars_in_a_lane,1);
    speed_for_cars_in_lane3=0.44704.*randi([50 70],No_of_cars_in_a_lane,1);
    speed_for_cars_in_lane4=0.44704.*randi([50 70],No_of_cars_in_a_lane,1);
    
    % Defining the vector to store the accleration of the cars
    accleration_for_cars_in_lane1 = -5 + (5+5)*rand(No_of_cars_in_a_lane,1);
    accleration_for_cars_in_lane2 = -5 + (5+5)*rand(No_of_cars_in_a_lane,1);
    accleration_for_cars_in_lane3 = -5 + (5+5)*rand(No_of_cars_in_a_lane,1);
    accleration_for_cars_in_lane4 = -5 + (5+5)*rand(No_of_cars_in_a_lane,1);
    
    % Defining the Matrix to store the velocity, accleration, position of the vehicle 
    position_matrix_1=[];
    position_matrix_2=[];
    position_matrix_3=[];
    position_matrix_4=[];
    
for timing_constraint=1:No_of_cars_in_a_lane
% Checking the difference between the position of the consecutive cars in each lane
% finding the distance between the cars in lane1
        if (timing_constraint==1)
            diff_in_1st_lane_positions = 0;
        else
            diff_in_1st_lane_positions = ypos_in_1st_lane(timing_constraint,1) - ypos_in_1st_lane(timing_constraint-1,1);
        end
        matrix1_row = [lane1pos ypos_in_1st_lane(timing_constraint,1) speed_for_cars_in_lane1(timing_constraint,1) accleration_for_cars_in_lane1(timing_constraint,1) diff_in_1st_lane_positions 0];
        position_matrix_1 = [position_matrix_1;matrix1_row];
% finding the distance between the cars in lane2
        if (timing_constraint==1)
            diff_in_2nd_lane_positions = 0;
        else
            diff_in_2nd_lane_positions = ypos_in_2nd_lane(timing_constraint,1) - ypos_in_2nd_lane(timing_constraint-1,1);
        end
        matrix2_row = [lane2pos ypos_in_2nd_lane(timing_constraint,1) speed_for_cars_in_lane2(timing_constraint,1) accleration_for_cars_in_lane2(timing_constraint,1) diff_in_2nd_lane_positions 0];
        position_matrix_2 = [position_matrix_2;matrix2_row];
% finding the distance between the cars in lane3
        if (timing_constraint==1)
            diff_in_3rd_lane_positions = 0;
        else
            diff_in_3rd_lane_positions = ypos_in_3rd_lane(timing_constraint,1) - ypos_in_3rd_lane(timing_constraint-1,1);
        end
        matrix3_row = [lane3pos ypos_in_3rd_lane(timing_constraint,1) speed_for_cars_in_lane3(timing_constraint,1) accleration_for_cars_in_lane3(timing_constraint,1) diff_in_3rd_lane_positions 0];
        position_matrix_3 = [position_matrix_3;matrix3_row];
% finding the distance between the cars in lane4
        if (timing_constraint==1)
            diff_in_4th_lane_positions = 0;
        else
            diff_in_4th_lane_positions = ypos_in_4th_lane(timing_constraint,1) - ypos_in_4th_lane(timing_constraint-1,1);
        end
        matrix4_row = [lane4pos ypos_in_4th_lane(timing_constraint,1) speed_for_cars_in_lane4(timing_constraint,1) accleration_for_cars_in_lane4(timing_constraint,1) diff_in_4th_lane_positions 0];
        position_matrix_4 = [position_matrix_4;matrix4_row];
        
% Code to calculate the new position and speed of the vehicle after 0.1 sec.
    for loop_to_average = 1:5   % loop to take the average of 5 samples as requested
%        for limit = 1:60       % loop to take the sample for 10 min
 
        for time = 0:1
%          Calculating the new speed, distance, acceleration 

% Code to arbitirarly provide entry and exit to cars at the position of ramps mentioned 
                lanecoordinatesX = [500,503,506,509];
                entry_ramp= [1500,2500,4000];
                if(random('exp',0.833) > 0.833)
                    entrypointX=randi(4,[1,1]);
                    entrypointY=randi(3,[1,1]);
%if the entry points and exit points in the entry ramp are free then generate new vehicles at the entry point                     
                    if (entrypointX == 1 && isempty(find(position_matrix_1(:,2) == entry_ramp(entrypointY), 1)))
                        position_matrix_1 = [position_matrix_1;[lanecoordinatesX(entrypointX) entry_ramp(entrypointY) randi([50 70],1,1) -5 + (5+5)*rand(1,1) 0 0]];
                    elseif (entrypointX == 2 && isempty(find(position_matrix_2(:,2) == entry_ramp(entrypointY), 1)))
                        position_matrix_2 = [position_matrix_2;[lanecoordinatesX(entrypointX) entry_ramp(entrypointY) randi([50 70],1,1) -5 + (5+5)*rand(1,1) 0 0]];
                    elseif (entrypointX == 3 && isempty(find(position_matrix_3(:,2) == entry_ramp(entrypointY), 1)))
                        position_matrix_3 = [position_matrix_3;[lanecoordinatesX(entrypointX) entry_ramp(entrypointY) randi([50 70],1,1) -5 + (5+5)*rand(1,1) 0 0]];
                    elseif (entrypointX == 4 && isempty(find(position_matrix_4(:,2) == entry_ramp(entrypointY), 1)))
                        position_matrix_4 = [position_matrix_4;[lanecoordinatesX(entrypointX) entry_ramp(entrypointY) randi([50 70],1,1) -5 + (5+5)*rand(1,1) 0 0]];
                    end
                end
%if the vehicle reaches the exit points in the exit ramp remove the
%vehicles at the exit point in all the lanes
                exitramp1 = [find(position_matrix_1(:,2) == 1550), find(position_matrix_1(:,2) == 2550), find(position_matrix_1(:,2) == 4050)];
                exitramp2 = [find(position_matrix_2(:,2) == 1550), find(position_matrix_2(:,2) == 2550), find(position_matrix_2(:,2) == 4050)];
                exitramp3 = [find(position_matrix_3(:,2) == 1550), find(position_matrix_3(:,2) == 2550), find(position_matrix_3(:,2) == 4050)];
                exitramp4 = [find(position_matrix_4(:,2) == 1550), find(position_matrix_4(:,2) == 2550), find(position_matrix_4(:,2) == 4050)];
                if(random('exp',0.833) > 0.833)
                    if (~isempty(exitramp1))
                        for j = 1:length(exitramp1)
                            position_matrix_1(exitramp1(j),:) = []; 
                        end
                    elseif (~isempty(exitramp2))
                        for j = 1:length(exitramp2)
                            position_matrix_2(exitramp2(j),:) = []; 
                        end
                    elseif (~isempty(exitramp3))
                        for j = 1:length(exitramp3)
                            position_matrix_3(exitramp3(j),:) = []; 
                        end
                    elseif (~isempty(exitramp4))
                        for j = 1:length(exitramp4)
                            position_matrix_4(exitramp4(j),:) = []; 
                        end
                    end
                end


% If the condition for safety distance is met then find the new speed and
% the position of the vehicle in each of the lanes
% Starting with lane2---> In this case we have to check for availability in
% lane 3 and lane 1
                flag = 0;
                for i = 1:size(position_matrix_2,1)
                            i = i-flag;                
                            new_speed2 = position_matrix_2(i,3) + (-1 + (1-(-1))*rand(1,1))*position_matrix_2(i,4);
                            position_matrix_2(i,2) = position_matrix_2(i,2) + (new_speed2*0.1);
                            if (i ~= 1)
                            position_matrix_2(i,5) = position_matrix_2(i,2) - position_matrix_2(i-1,2);
                            end
%If the distance between the two vehicles is less than 10,
                            if (i ~= 1)
                                if(position_matrix_2(i,2) - position_matrix_2(i-1,2) < 10)
                                     % checkk if the nearby lane is free or not 
                                      lanecheck3 = find(position_matrix_3(:,2)<=position_matrix_2(i,2)+10 & position_matrix_3(:,2)>=position_matrix_2(i,2)-10);
                                      lanecheck1 = find(position_matrix_1(:,2)<=position_matrix_2(i,2)+10 & position_matrix_1(:,2)>=position_matrix_2(i,2)-10);
                                     if(isempty(lanecheck3))
                                        position_matrix_3 = [position_matrix_3; [position_matrix_3(1,1) position_matrix_2(i,2) position_matrix_2(i,3) position_matrix_2(i,4) position_matrix_2(i,5) 0]];
%                                       Sort rows based on Y values  
                                        position_matrix_3 = sortrows(position_matrix_3,2);
%                                       Remove the entry from first lane
                                        position_matrix_2(i,:) = [];
                                        flag = flag+1;
                          
                                     elseif(isempty(lanecheck1))
                                        position_matrix_1 = [position_matrix_1; [position_matrix_1(1,1) position_matrix_2(i,2) position_matrix_2(i,3) position_matrix_2(i,4) position_matrix_2(i,5) 0]];
%                                       Sort the position of the cars in the first lane  
                                        position_matrix_1 = sortrows(position_matrix_1,2);
%                                       Remove the entry from first lane
                                        position_matrix_2(i,:) = [];
                                        flag = flag+1;                         
                                     else
                                        factor=(-0.75+sqrt(0.5625+0.02804*(position_matrix_2(i,2) - position_matrix_2(i-1,2))))/0.01408;
                                        position_matrix_2(i,3) = min(position_matrix_2(i-1,3),factor); % Min value of the speed or the factor for car following model
                                     end
                                end
                                
                            end 
                end
% After the completion of freeway, lane changing and carfollowing for first lane sort the cars in the lane                
        position_matrix_2 = sortrows(position_matrix_2);
        position_matrix_1 = sortrows(position_matrix_1);
        position_matrix_3 = sortrows(position_matrix_3);

% Same process for lane1---> Here we need to check only lane two
% availablity 

                  flag = 0;
    for i = 1:size(position_matrix_1,1)
                i = i - flag;
                new_speed1 = position_matrix_1(i,3) + (-1 + (1-(-1))*rand(1,1))*position_matrix_1(i,4);
                position_matrix_1(i,2) = position_matrix_1(i,2) + (new_speed1*0.1);
                if (i ~= 1)
                position_matrix_1(i,5) = position_matrix_1(i,2) - position_matrix_1(i-1,2);
                end
                if (i ~= 1)
                    if(position_matrix_1(i,2) - position_matrix_1(i-1,2) < 10)
                        lanecheck2 = find(position_matrix_2(:,2)<=position_matrix_1(i,2)+10 & position_matrix_2(:,2)>=position_matrix_1(i,2)-10);
                        if(isempty(lanecheck2))
                            position_matrix_2 = [position_matrix_2; [position_matrix_1(1,1) position_matrix_1(i,2) position_matrix_1(i,3) position_matrix_1(i,4) position_matrix_1(i,5) 0]];  
                            position_matrix_2 = sortrows(position_matrix_2,2);
                            position_matrix_1(i,:) = [];
                            flag = flag + 1;
                        else
                            factor=(-0.75+sqrt(0.5625+0.02804*(position_matrix_1(i,2) - position_matrix_1(i-1,2))))/0.01408;
                            position_matrix_1(i,3) = min(position_matrix_1(i-1,3),factor);
                        end
                    end    
                end
    end
    position_matrix_1 = sortrows(position_matrix_1);
    position_matrix_2 = sortrows(position_matrix_2);
  
% Same processing for lane 4
                  flag = 0;
    for i = 1:size(position_matrix_4,1)
                i = i - flag;
                new_speed1 = position_matrix_4(i,3) + (-1 + (1-(-1))*rand(1,1))*position_matrix_4(i,4);
                position_matrix_4(i,2) = position_matrix_4(i,2) + (new_speed1*0.1);
                if (i ~= 1)
                position_matrix_4(i,5) = position_matrix_4(i,2) - position_matrix_4(i-1,2);
                end
                if (i ~= 1)
                    if(position_matrix_4(i,2) - position_matrix_4(i-1,2) < 10)
                        lanecheck3 = find(position_matrix_3(:,2)<=position_matrix_4(i,2)+10 & position_matrix_3(:,2)>=position_matrix_4(i,2)-10);
                        if(isempty(lanecheck3))
                            position_matrix_3 = [position_matrix_3; [position_matrix_4(1,1) position_matrix_4(i,2) position_matrix_4(i,3) position_matrix_4(i,4) position_matrix_4(i,5) 0]];
                            position_matrix_3 = sortrows(position_matrix_3,2);
                            position_matrix_4(i,:) = [];
                            flag = flag + 1;
                        else
                            factor=(-0.75+sqrt(0.5625+0.02804*(position_matrix_4(i,2) - position_matrix_4(i-1,2))))/0.01408;
                            position_matrix_4(i,3) = min(position_matrix_4(i-1,3),11.98);
                        end
                    end    
                end
    end
    position_matrix_4 = sortrows(position_matrix_4);
    position_matrix_3 = sortrows(position_matrix_3);

% Same processing for lane 3 
                flag = 0;
                for i = 1:size(position_matrix_3,1)  
                            i = i-flag;                
                            new_speed2 = position_matrix_3(i,3) + (-1 + (1-(-1))*rand(1,1))*position_matrix_3(i,4);
                            position_matrix_3(i,2) = position_matrix_3(i,2) + (new_speed2*0.1);
                            if (i ~= 1)
                            position_matrix_3(i,5) = position_matrix_3(i,2) - position_matrix_3(i-1,2);
                            end
                            if (i ~= 1)
                                if(position_matrix_3(i,2) - position_matrix_3(i-1,2) < 10)
                                      lanecheck3 = find(position_matrix_4(:,2)<=position_matrix_3(i,2)+10 & position_matrix_4(:,2)>=position_matrix_3(i,2)-10);
                                      lanecheck1 = find(position_matrix_2(:,2)<=position_matrix_3(i,2)+10 & position_matrix_2(:,2)>=position_matrix_3(i,2)-10);
                                     if(isempty(lanecheck3))
                                        position_matrix_4 = [position_matrix_4; [position_matrix_4(1,1) position_matrix_3(i,2) position_matrix_3(i,3) position_matrix_3(i,4) position_matrix_3(i,5) 0]]; 
                                        position_matrix_4 = sortrows(position_matrix_4,2);
                                        position_matrix_3(i,:) = [];
                                        flag = flag+1;
                          
                                     elseif(isempty(lanecheck1))
                                        position_matrix_2 = [position_matrix_2; [position_matrix_2(1,1) position_matrix_3(i,2) position_matrix_3(i,3) position_matrix_3(i,4) position_matrix_3(i,5) 0]]; 
                                        position_matrix_2 = sortrows(position_matrix_2,2);
                                        position_matrix_3(i,:) = [];
                                        flag = flag+1;                           
                                     else
                                        factor=(-0.75+sqrt(0.5625+0.02804*(position_matrix_3(i,2) - position_matrix_3(i-1,2))))/0.01408;
                                        position_matrix_3(i,3) = min(position_matrix_3(i-1,3),factor);
                                     end
                                end
                                
                            end 
                end
        position_matrix_3 = sortrows(position_matrix_3);
        position_matrix_2 = sortrows(position_matrix_2);
        position_matrix_4 = sortrows(position_matrix_4);
% Boundary effect- if a car reaches the end point of the lane, then reset
% it back to the same lane
%              Lane 1
                if(max(position_matrix_1(:,2))>5000)
                    x = find(position_matrix_1(:,2) > 5000);
                    dist = 0;
                    diff = position_matrix_1(1,2);
                    for k = 1:length(x)
                        position_matrix_1(x(k),:) = []; 
                        position_matrix_1 = [position_matrix_1; [position_matrix_1(1,1) dist randi([50 70],1,1) -5 + (5+5)*rand(1,1) diff 0]];
                        position_matrix_1 = sortrows(position_matrix_1,2);
                        dist = dist + 20;
                        diff = diff + 20;
                    end
                end    
%              Lane 2
                if(max(position_matrix_2(:,2))>5000)
                    x = find(position_matrix_2(:,2) > 5000);
                    dist = 0;
                    diff = position_matrix_2(1,2);
                    for k = 1:length(x)
                        position_matrix_2(x(k),:) = []; 
                        position_matrix_2 = [position_matrix_2; [position_matrix_1(2,1) dist randi([50 70],1,1) -5 + (5+5)*rand(1,1) diff 0]];
                        position_matrix_2 = sortrows(position_matrix_2,2);
                        dist = dist + 20;
                        diff = diff + 20;
                    end
                end
%              Lane 3
                if(max(position_matrix_3(:,2))>5000)
                    x = find(position_matrix_3(:,2) > 5000);
                    dist = 0;
                    diff = position_matrix_3(1,2);
                    for k = 1:length(x)
                        position_matrix_3(x(k),:) = []; 
                        position_matrix_3 = [position_matrix_3; [position_matrix_3(2,1) dist randi([50 70],1,1) -5 + (5+5)*rand(1,1) diff 0]];
                        position_matrix_3 = sortrows(position_matrix_3,2);
                        dist = dist + 20;
                        diff = diff + 20;
                    end
                end
%              Lane 4
                if(max(position_matrix_4(:,2))>5000)
                    x = find(position_matrix_4(:,2) > 5000);
                    dist = 0;
                    diff = position_matrix_4(1,2);
                    for k = 1:length(x)
                        position_matrix_4(x(k),:) = []; 
                        position_matrix_4 = [position_matrix_4; [position_matrix_4(2,1) dist randi([50 70],1,1) -5 + (5+5)*rand(1,1) diff 0]];
                        position_matrix_4 = sortrows(position_matrix_4,2);
                        dist = dist + 20;
                        diff = diff + 20;
                    end
                end
          end
                position_matrix_1 = sortrows(position_matrix_1,2);
                position_matrix_2 = sortrows(position_matrix_2,2);
                position_matrix_3 = sortrows(position_matrix_3,2);
                position_matrix_4 = sortrows(position_matrix_4,2);
        
%   Data generationg for question 1    
   
        
            check = find(position_matrix_1(:,6) == 1);
    if(isempty(check))
        check = find(position_matrix_2(:,6) == 1);
        if(isempty(check))
            check = find(position_matrix_3(:,6) == 1);
            if(isempty(check))
                check = find(position_matrix_4(:,6) == 1);
                lane_num = position_matrix_4;
            else
                lane_num = position_matrix_3;
            end
        else
            lane_num = position_matrix_2;
        end
    else
        lane_num = position_matrix_1;
    end  
%  Calculating number of nodes in Lane1
    nodes_1 = 0;
    vehicles_in_lane1 = 0;
    for i = 1:size(position_matrix_1,1)
        Coord = [lane_num(check,1),lane_num(check,2);position_matrix_1(i,1),position_matrix_1(i,2)];
        dist  = pdist(Coord,'euclidean'); 
        if (dist <= Communication_range)
            nodes_1 = [nodes_1,i] ;
            vehicles_in_lane1 = vehicles_in_lane1 + 1;
        end
    end

%  Calculating number of nodes in Lane2
    nodes_2 = 0;
    vehicles_in_lane2 = 0;
    for i = 1:size(position_matrix_2,1)
        Coord = [lane_num(check,1),lane_num(check,2);position_matrix_2(i,1),position_matrix_2(i,2)];
        dist  = pdist(Coord,'euclidean'); 
        if (dist < Communication_range)
            nodes_2 = [nodes_2,i] ;
            vehicles_in_lane2 = vehicles_in_lane2 + 1;
        end
    end
%  Calculating number of nodes in Lane3

    nodes_3 = 0;
    vehicles_in_lane3 = 0;
    for i = 1:size(position_matrix_3,1)
        Coord = [lane_num(check,1),lane_num(check,2);position_matrix_3(i,1),position_matrix_3(i,2)];
        dist  = pdist(Coord,'euclidean'); 
        if (dist < 50)
            nodes_3 = [nodes_3,i];
            vehicles_in_lane3 = vehicles_in_lane3 + 1;
        end
    end
%  Calculating number of nodes in Lane4
    nodes_4 = 0;
    vehicles_in_lane4 = 0;
    for i = 1:size(position_matrix_4,1)
        Coord = [lane_num(check,1),lane_num(check,2);position_matrix_4(i,1),position_matrix_4(i,2)];
        dist  = pdist(Coord,'euclidean'); 
        if (dist < 50)
            nodes_4 = [nodes_4, i];
            vehicles_in_lane4 = vehicles_in_lane4 + 1;
        end
    end    
    connected_vehicles = length(vehicles_in_lane1)+vehicles_in_lane2+vehicles_in_lane3+vehicles_in_lane4;
    total_connected_vehicles = total_connected_vehicles + connected_vehicles;

%   Data generationg for question 2
        nodes_1_intersect = intersect(nodes_1,nodes_1_intersect);
        nodes_2_intersect = intersect(nodes_2,nodes_2_intersect);
        nodes_3_intersect = intersect(nodes_3,nodes_3_intersect);
        nodes_4_intersect = intersect(nodes_4,nodes_4_intersect);
        inter_length = length(nodes_1_intersect) + length(nodes_2_intersect) +length(nodes_3_intersect)+length(nodes_4_intersect);
        if(inter_length >= 3)
            count = count + 1;
        else
            counter = counter + 1;
        end
        if(time == 5)
        
            neighbour = count + counter;
            same_comm_neighbors = [same_comm_neighbors,neighbour];
        
        end    
        end 
        
    avg_vehicles = total_connected_vehicles/(limit/increment);
    avg_nodes = [avg_nodes;avg_vehicles];
        
    time_dur_3_neighbors = [time_dur_3_neighbors,(count*0.1)];
    

        end
%   Data generationg for question 2
    avg_V2V = [avg_V2V,mean(avg_nodes)];
    avg_3_neighbors = [avg_3_neighbors,mean(time_dur_3_neighbors)];
    
    avg_same_comm_neighbors = [avg_same_comm_neighbors,mean(same_comm_neighbors)];
    
end

figure(1)
title('Average No of nodes in the V2V network Vs Traffic Density');
xlabel('Traffic Density');
ylabel('Avg V2V Connectivity');
hold on
plot(trafficdensity,avg_V2V,'k');

figure(2)
title('Average duration - 3 neighbors connected vs Traffic Density');
xlabel('Traffic Density');
ylabel('Average duration');
hold on;
plot(trafficdensity,avg_3_neighbors,'K');


figure(3)
title('Average number of same communication neighbors Vs Traffic Density');
xlabel('Traffic Density');
ylabel('Avg same communication neighbours');
hold on;
plot(trafficdensity, avg_same_comm_neighbors,'k');



