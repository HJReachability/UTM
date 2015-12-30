function all_target_locations = generate_target_locatiosn(N, board_size)
    all_target_locations = cell(N, 1);
    if N == 2
        all_target_locations{1} = [0 0];
        all_target_locations{2} = [board_size board_size];
    elseif N == 3
        all_target_locations{1} = [0 board_size];
        all_target_locations{2} = [0 0];
        all_target_locations{3} = [board_size board_size/2];
    elseif N == 4
        all_target_locations{1} = [0 0];
        all_target_locations{2} = [0 board_size];
        all_target_locations{3} = [board_size 0];
        all_target_locations{4} = [board_size board_size];
    elseif N == 5
        all_target_locations{1} = [0 0];
        all_target_locations{2} = [0 board_size];
        all_target_locations{3} = [board_size 0];
        all_target_locations{4} = [board_size board_size];
        all_target_locations{5} = [board_size/2 board_size];
    elseif N == 6
        all_target_locations{1} = [0 0];
        all_target_locations{2} = [0 board_size];
        all_target_locations{3} = [board_size 0];
        all_target_locations{4} = [board_size board_size];
        all_target_locations{5} = [board_size/2 board_size];
        all_target_locations{6} = [board_size/2 0];
    else
        error('N larger than 6. generate_target_locations unimplemented.');
    end

end