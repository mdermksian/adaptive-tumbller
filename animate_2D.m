function[] = animate_2D(t, y)

%     dt = 0.001;

    n = length(t);

    % Mechanical parameters
    L_com = 0.04;
    L_full = 0.11;
    r = 0.0335;
    w = 0.005;

    R_T = @(t) [cos(t) -sin(t); sin(t) cos(t)].';

    cart_coord_base = [r/2 0; -r/2 0; -r/2 r; r/2 r];

    pend_coord_base = [w 0; -w 0; -w L_full; w L_full];

    cart_coord = cart_coord_base + [y(1,1) 0];
    pend_coord = pend_coord_base * R_T(y(1,3)) + [y(1,1) r/2];

    f = figure();
    set(f, 'Position', [1 41 1536 748.8]);
    % set(f, 'Position', get(0, 'Screensize'));
    set(f, 'DoubleBuffer', 'on');
    hold on;
    axis equal;
    axis([-0.2 1.4 -0.15 0.15]);
    plot([-0.2 1.4], [0 0], '-k');
    p_cart = patch(cart_coord(:,1), cart_coord(:,2), 'g');
    p_pend = patch(pend_coord(:,1), pend_coord(:,2), 'b');
    tbox = text(0, 0.2, strcat(num2str(t(1)), ' s'), 'FontSize', 10);

    disp('Press any key to animate...');
    pause;

    % Animate platform
    for i = 1 : n-1
        dt = t(i+1) - t(i);
        start = tic;

        x = y(i, 1);
        theta = y(i, 3);

        cart_coord = cart_coord_base + [x 0];
        pend_coord = pend_coord_base * R_T(theta) + [x r/2];

        p_cart.XData = cart_coord(:, 1);
        p_cart.YData = cart_coord(:, 2);
        p_pend.XData = pend_coord(:, 1);
        p_pend.YData = pend_coord(:, 2);
        tbox.String = strcat(num2str(t(i)), ' s');
        drawnow limitrate;

        finish = toc(start);
        if(finish < dt)
            pause(dt - finish);
        end
    end

    disp('Press any key to close figure...');
    pause;
    close(f);
end
