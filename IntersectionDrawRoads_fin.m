axis([0 1000 0 1000]);
[x,y] = ginput(2);
line(x,y,'color','k'); 
hold on

[p,q] = ginput(2); % Take in the inputs
line(p,q,'color','k'); 

a1 = linspace(x(1),x(2),10);
b1 = linspace(y(1),y(2),10);

a2 = linspace(p(1),p(2),10);
b2 = linspace(q(1),q(2),10);

for i = 1:1000
    t1x = (rem(i,10)+1);
    t1y = (rem(i,10)+1);
    t2x = (rem(i,10)+1);
    t2y = (rem(i,10)+1);

h1 = plot(a1(t1x),b1(t1y),'*r') %Properties of vehicle1
h2 = plot(a2(t2x),b2(t2y),'*g') %Properties of vehicle2

m  = floor(sqrt((a1(t1x)- a2(t2x))^2+(b1(t1y)- b2(t2y))^2))
if m<100    %Condition to check if the communication range for the V2V communiction is less than 100
h3 = plot([a1(t1x) a2(t2x)],[b1(t1y) b2(t2y)])
pause(0.3);
set(h3,'Visible','off')
set(h1,'Visible','off')
set(h2,'Visible','off')
end
pause(0.5);
set(h1,'Visible','off')
set(h2,'Visible','off')


end
