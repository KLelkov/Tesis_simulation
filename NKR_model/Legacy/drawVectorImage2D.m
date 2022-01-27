function frame = drawVectorImage2D(vectorImage, position, fig, color)
    figure(fig);
    frame = [];
    for i=2:length(vectorImage)
        frame(end+1) = line(position(1) + vectorImage(i-1:i,2), position(2) + vectorImage(i-1:i,1), 'Color', color, 'LineWidth', 2);
    end
end

