clear
close all
clc

position = 1:628;
%for i = 1:628
%    x = sin((i-1)/100);
%    fprintf("%1.4f,\n", x);  % speed in 1000 radians per second
%end

speed = linspace(-50, 16, 1024);


for i = 1:1024
    fprintf("%2.4f, \n", speed(i));
end