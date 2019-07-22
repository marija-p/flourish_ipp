function [h_pyr] = draw_pyramid(P,l,w,h) 
% PYRAMID will accept four inputs. P0 is an array of 3 scalar numbers for 
% the origin (x, y, and z). l is the length of the box in the x-direction, 
% w is the width of the box in the y-direction, and h is the height of the 
% box in the z-direction. The functin will draw a square pyramid. 
% Input: Four inputs, an array of the point of origin, a length, width, and 
% height. 
% Output: A pyramid drawn with a set transparency and different colors for 
% the faces 

% Colour observation pyramid based on its height
colour = [max(1-h/8,0) 0 min(1,h/8)];

x = [P(1),P(1)+l,P(1)+l,P(1)]; 
y = [P(2),P(2),P(2)-w,P(2)-w]; 
z = [P(3),P(3),P(3),P(3)]; 
h_pyr(1) = fill3(x, y, z , colour); hold on  
x2 = [P(1),P(1)+l,P(1)+ l/2]; 
y2 = [P(2),P(2),P(2)-w/2]; 
z2 = [P(3),P(3),P(3)+h]; 
h_pyr(2) = fill3(x2, y2, z2, colour); hold on
x3 = [P(1)+l,P(1)+l,P(1) + l/2]; 
y3 = [P(2), P(2)-w,P(2)- w/2]; 
z3 = [P(3),P(3),P(3)+h]; 
h_pyr(3) = fill3(x3, y3, z3, colour); hold on 
x4 = [P(1)+l,P(1),P(1)+ l/2]; 
y4 = [P(2)-w,P(2)-w,P(2)- w/2]; 
z4 = [P(3),P(3),P(3)+h];  
h_pyr(4) = fill3(x4,y4,z4, colour); hold on 
x5 = [P(1),P(1),P(1) + l/2]; 
y5 = [P(2),P(2)-w,P(2)- w/2]; 
z5 = [P(3),P(3),P(3)+h];  
h_pyr(5) = fill3(x5,y5,z5,colour);

%alpha(h_pyr(1), 0.3);
%alpha(h_pyr(2:5), 0.05);

alpha(h_pyr(1), 0);
alpha(h_pyr(2:5), 0);