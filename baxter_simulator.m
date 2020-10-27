function baxter_simulator(path)
    [left, right] = mdl_baxter('');
    
    path = squeeze(path);
    for i = 1:length(path)-1
        if mod(i,7) == 1
            path2(round((i+6)/7),:) = path(i:i+6);
        end
    end
    
    q = cell2mat(path2);
%     q = path;
    
    
    

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
    E = right.fkine([0.5 0.5 0.5 0.5 0.5 0.5 0.5]).t;
    scatter3(E(1),E(2),E(3),'m','filled');
    for i =1:size(final_path,1)
        E = right.fkine(final_path(i,:)).t;
        scatter3(E(1),E(2),E(3),'b','filled');
        drawnow;
        mov(:,i) = getframe(gcf);
        writeVideo(v,mov(:,i))
        right.animate(final_path(i,:))
    end
    writeVideo(v, getframe(gcf))    
    close(v)
    
    

end

