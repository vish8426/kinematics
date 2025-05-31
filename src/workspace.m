function [] = workspace(theta1_min, theta1_max, theta2_min, theta2_max)
    
    samples = 200;
    a1 = 0.28;
    a2 = 0.35;
    
    theta1_start_end = pi*[theta1_min, theta1_max]/180;
    theta2_start_end = pi*[theta2_min, theta2_max]/180;
    
    theta1 = pi*linspace(theta1_min, theta1_max, samples)/180;
    theta2 = pi*linspace(theta2_min, theta2_max, samples)/180;
    
    x = zeros(2*length(theta1_start_end),length(theta2));
    y = zeros(2-length(theta1_start_end),length(theta2));
    
    for t = 1:2
        for ith1 = 1:length(theta1)
            x(t,ith1) = a1*cos(theta1(ith1)) + a2*cos(theta1(ith1)+theta2_start_end(t));
            y(t,ith1) = a1*sin(theta1(ith1)) + a2*sin(theta1(ith1)+theta2_start_end(t));
        end
        
        for ith2 = 1:length(theta1)
            x(t+2,ith2) = a1*cos(theta1_start_end(t)) + a2*cos(theta1_start_end(t)+theta2(ith2));
            y(t+2,ith2) = a1*sin(theta1_start_end(t)) + a2*sin(theta1_start_end(t)+theta2(ith2));
        end
    end

    x = x';
    y = y';

    plot(x(:,1),y(:,1),'k')
    hold on
    plot(x(:,2),y(:,2),'k')
    plot(x(:,3),y(:,3),'k')
    plot(x(:,4),y(:,4),'k')
end