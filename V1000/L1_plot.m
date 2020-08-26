%L1_plot
    prev_WP=[40,100]*1e7;
    next_WP=[40,100.01]*1e7;

    figure
    hold on
    plot(loc.lon(1:end-1),loc.lat(1:end-1),'O')
    plot(current_loc_out(:,2),current_loc_out(:,1),'*')
    figure
    plot(Xe_out(:,2),Xe_out(:,1))
    
