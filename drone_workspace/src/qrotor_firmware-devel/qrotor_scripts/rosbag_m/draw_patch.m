function draw_patch(x,y)


for i = 1:size(x,1)
    
    X = [x(i,1), x(i,2), x(i,2), x(i,1)];
    Y = [y(1), y(1), y(2), y(2)];
    patch(X, Y,'y','FaceAlpha',0.25, 'EdgeColor', 'none');
end



end