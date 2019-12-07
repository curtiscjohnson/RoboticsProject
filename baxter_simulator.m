function baxter_simulator(path)
    close all
    [left, right] = mdl_baxter('');
    
    path = squeeze(path);
    for i = 1:length(path)-1
        if mod(i,7) == 1
            path2(round((i+6)/7),:) = path(i:i+6);
        end
    end
    
    q = cell2mat(path2);
    
    
    
    

    obst_location = [1.0 -0.5 0.1];
    obst_radius = 0.3;
    [x,y,z] = sphere;
    % Scale to desire radius.
    x = x * obst_radius;
    y = y * obst_radius;
    z = z * obst_radius;
    % Plot as surface.
    close all
    figure()
    surf(x+obst_location(1),y+obst_location(2),z+obst_location(3), 'FaceAlpha', 0.2)
    hold on
    
    v = VideoWriter('movie');
    v.Quality = 100;
    open(v)
    
    final_path = Interpolate_Matrix(q,5)
    right.plot(final_path(1,:))
    for i =1:size(final_path,1)
        drawnow;
        mov(:,i) = getframe(gcf);
        writeVideo(v,mov(:,i))
        right.animate(final_path(i,:))
        pause(0.01)
    end
    writeVideo(v, getframe(gcf))    
    close(v)
    
    

end

