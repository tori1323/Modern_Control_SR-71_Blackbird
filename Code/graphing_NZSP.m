function [] = graphing_NZSP(table,Title)
    
    % Plot the State Graphs
    figure
    hold on
    sgtitle(Title + " States")
    subplot(5,1,1)
    plot(table.t(1,:),(180/pi)*table.y(1,:))
    ylabel('State \beta [degs]')
    subplot(5,1,2)
    plot(table.t(1,:),(180/pi)*table.y(2,:))
    ylabel('State p [degs/s]')
    subplot(5,1,3)
    plot(table.t(1,:),(180/pi)*table.y(3,:))
    ylabel('State r [degs/s]')
    subplot(5,1,4)
    plot(table.t(1,:),(180/pi)*table.y(4,:))
    ylabel('State \phi [degs]')
    subplot(5,1,5)
    hold on
    plot(table.t(1,:),(180/pi)*table.y(5,:))
    plot(table.t(1,:),(180/pi)*table.track(1,:))
    legend('State','Desired Step Output')
    hold off
    xlabel('Time [s]')
    ylabel('State \psi [degs]')
    hold off
    
    % Plot the Control Graphs
    figure
    hold on
    sgtitle(Title + " Controls")
    subplot(2,1,1)
    plot(table.t(1,:),(180/pi)*table.y(6,:))
    ylabel('\delta_a [degs]')
    subplot(2,1,2)
    plot(table.t(1,:),(180/pi)*table.y(7,:))
    xlabel('Time [s]')
    ylabel('\delta_r [degs]')
    hold off
    
    % Plot the Commands Graphs
    figure
    hold on
    sgtitle(Title + " Commands")
    subplot(2,1,1)
    plot(table.t(1,:),(180/pi)*table.y(8,:))
    ylabel('\delta_a [degs]')
    subplot(2,1,2)
    plot(table.t(1,:),(180/pi)*table.y(9,:))
    xlabel('Time [s]')
    ylabel('\delta_r [degs]')
    hold off
    
    % Plot the Rates Graphs
    figure
    hold on
    sgtitle(Title + " Rates")
    subplot(2,1,1)
    plot(table.t(1,:),(180/pi)*table.y(10,:))
    ylabel('\delta_a [degs/s]')
    subplot(2,1,2)
    plot(table.t(1,:),(180/pi)*table.y(11,:))
    xlabel('Time [s]')
    ylabel('\delta_r [degs/s]')
    hold off
end