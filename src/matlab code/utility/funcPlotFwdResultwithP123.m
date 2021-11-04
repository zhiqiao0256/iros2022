function []=funcPlotFwdResultwithP123(trainSet,par_set)
    x = 0 : .01 : 2 * pi;
    xx=x-deg2rad(par_set.p1_angle)+pi/3;
    rr=par_set.trianlge_length/sqrt(3)*cos(pi/3)./(cos(mod(xx,2*pi/3)-pi/3));
    edge_full=[];
    edge_full=[rr.*cos(x);rr.*sin(x);0*rr];
    figure('Position',[100;100;600;600])
    color_set={'r','b','k'};
    for i =1:3
        scatter3(par_set.r_p{i}(1),par_set.r_p{i}(2),par_set.r_p{i}(3),'*','MarkerEdgeColor',color_set{i})
        hold on
    end
    scatter3(trainSet.tip_exp(1,2),trainSet.tip_exp(1,3),trainSet.tip_exp(1,4),'g','*','LineWidth',4)
    hold on
    scatter3(trainSet.tip_exp(end,2),trainSet.tip_exp(end,3),trainSet.tip_exp(end,4),'c','*','LineWidth',4)
    hold on
    scatter3(trainSet.x_y_edge(:,1),trainSet.x_y_edge(:,2),trainSet.x_y_edge(:,3),'k')
    hold on
    scatter3(trainSet.tip_exp(:,2),trainSet.tip_exp(:,3),trainSet.tip_exp(:,4),'r','hexagram')
    hold on
    scatter3(0,0,0,'r','diamond')
    hold on
    plot3(edge_full(1,:),edge_full(2,:),edge_full(3,:),'LineStyle',':','Color','k','LineWidth',2)
    hold on
    plot3(trainSet.p_1_top(:,1),trainSet.p_1_top(:,2),trainSet.p_1_top(:,3),'r')
    hold on
        plot3(trainSet.p_2_top(:,1),trainSet.p_2_top(:,2),trainSet.p_2_top(:,3),'b')
    hold on
        plot3(trainSet.p_3_top(:,1),trainSet.p_3_top(:,2),trainSet.p_3_top(:,3),'k')
    hold on
    legend('P1_b','P2_b','P3_b','Start','End','Intersection point','Top center','Bottom center','Inextensible edge',...
        'P1_t','P]2_t','P3_t','Location','northwest')
    xlabel('X-axis (m)')
    ylabel('Y-axis (m)')
    zlabel('Z-axis (m)')
    xlim([-0.1,0.1])
    ylim([-0.1,.1])
    zlim([0,0.2])
    view([1,-1,1])
end