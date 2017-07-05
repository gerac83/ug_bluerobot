function show_R(x,R)

hold on
quiver3(x(1), x(2), x(3), R(1,1), R(2,1), R(3,1), 'r')
quiver3(x(1), x(2), x(3), R(1,2), R(2,2), R(3,2), 'g')
quiver3(x(1), x(2), x(3), R(1,3), R(2,3), R(3,3), 'b')
hold off

end
