axis([0 1000 0 1000]);hold on
nodes = 5;
PauseTime = 0.00005; % Delay to decide the speed of the vehicle
[x,y] = ginput(nodes);
plot(x,y,'--rs','LineWidth',2,'MarkerEdgeColor','b','MarkerFaceColor','b','MarkerSize',4)
[p,q] = ginput(2);


plot(p(1),q(1),'*k')  % Position of the RSU1
plot(p(2),q(2),'*k'); % Position of the RSU2

xp = x(1);
yp = y(1);
m = 0; 

for loop = 1:20
    for i = 2:nodes
        xc = x(i); 
        yc = y(i); 
        xp = x(i-1);
        yp = y(i-1);
        m = floor(sqrt((xp-xc)*(xp-xc)+(yp-yc)*(yp-yc)))
        a1 = linspace(xp,xc,m);
        b1 = linspace(yp,yc,m);
            for j = 2:m
            h1= plot(a1(j),b1(j),'--rs','LineWidth',0.22,'MarkerEdgeColor','g','MarkerFaceColor','g','MarkerSize',5)
            % The roadway is plotted in red dotted line 

            pt  = [a1(j);b1(j)];
            pt1 = [p(1);q(1)];
            pt2 = [p(2);q(2)];
            d1 = norm(pt-pt1); 
            d2 = norm(pt-pt2); 
            %checking if the distance is less than 100, the specified
            %distance for V2I communication(Communication Range)
            if(d1<=100 && d1<d2)
                   h2 = plot([a1(j) p(1)],[b1(j) q(1)])
                   pause(PauseTime);
                   set(h2,'Visible','off');
                   continue
            elseif(d2<=100)
                   h3 = plot([a1(j) p(2)],[b1(j) q(2)])
                   pause(PauseTime);
                   set(h3,'Visible','off');
                   continue;                
            end
                    
            
            pause(PauseTime);           
            set(h1,'Visible','off');

            end
    end
end
