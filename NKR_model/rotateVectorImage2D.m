function rotatedImage = rotateVectorImage2D(vectorImage, angle)
    % Apply rotation angle (in radians) to the vector image
    rotatedImage = vectorImage;
    for i=1:length(vectorImage)
        rotatedImage(i,1) = vectorImage(i,1)*cos(angle) - vectorImage(i,2)*sin(angle);
        rotatedImage(i,2) = vectorImage(i,1)*sin(angle) + vectorImage(i,2)*cos(angle);
    end
end