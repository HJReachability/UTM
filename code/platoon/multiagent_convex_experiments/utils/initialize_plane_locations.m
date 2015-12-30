function all_planes = initialize_plane_locations(N, board_size, ...
    capture_radius, speed)

    all_planes = [];
    
    x = board_size * rand(1, 1);
    y = board_size * rand(1, 1);
    ang = 2 * pi * rand(1, 1);

    pl = Plane([x y wrapTo2Pi(ang)]);
    pl.speed = speed;
    all_planes = [all_planes pl];
    
    for i=2:N   
        while true
            x = board_size * rand(1, 1);
            y = board_size * rand(1, 1);
            location_valid = true;
            for j = 1:i-1
                x_other = all_planes(j).x(1);
                y_other = all_planes(j).x(2);
                if ((x - x_other)^2 + (y - y_other)^2) <= ...
                        (capture_radius + 1)^2
                    location_valid = false;
                    break
                end
            end
            if location_valid
                break
            end
        end
        ang = 2 * pi * rand(1, 1);
        pl = Plane([x y wrapTo2Pi(ang)]);
        pl.speed = speed;
        all_planes = [all_planes pl];
    end

end