function lk
    clear;
    frm1 = double(rgb2gray(imread('frame_0001.png')))./255;
    frm2 = double(rgb2gray(imread('frame_0002.png')))./255;
    [Vx, Vy] = calc_OF_lk(frm1, frm2, 1);
    display_plot(Vx, Vy);
    display_hsv(Vx, Vy);
end


%Calculate Optical Flow with Lucas-Kanade algorithm.
function [Vx, Vy] = calc_OF_lk(frame1, frame2, window_size)

    % Calculate derivates
    dx = conv2(frame1, 0.25 * [-1 1; -1 1]) + conv2(frame2, 0.25 * [-1 1; -1 1]);
    dy = conv2(frame1, 0.25 * [-1 -1; 1 1]) + conv2(frame2, 0.25 * [-1 -1; 1 1]);
    dt = conv2(frame1, 0.25 * ones(2)) + conv2(frame2, -0.25 * ones(2));

    % Calculate optical flow
    window_center = floor(window_size / 2);
    img_size = size(frame1);
    Vx = zeros(img_size);
    Vy = zeros(img_size);
    for i = window_center + 1:img_size(1) - window_center
      for j = window_center + 1:img_size(2) - window_center
        % Get values for current window
        dx_window = dx(i - window_center:i + window_center, j - window_center:j + window_center);
        dy_window = dy(i - window_center:i + window_center, j - window_center:j + window_center);
        dt_window = dt(i - window_center:i + window_center, j - window_center:j + window_center);

        dx_window = dx_window';
        dy_window = dy_window';
        dt_window = dt_window';

        A = [dx_window(:) dy_window(:)];
        b = -dt_window(:);

        V = pinv(A' * A) * A' * b;

        Vx(i, j) = V(1);
        Vy(i, j) = V(2);
      end
    end
end

%Display the OF as a plot of vectors
function display_plot(Vx,Vy)
    figure
    axis equal
    quiver(impyramid(impyramid(medfilt2(flipud(Vx), [5 5]), 'reduce'), 'reduce'), -impyramid(impyramid(medfilt2(flipud(Vy), [5 5]), 'reduce'), 'reduce'));
end

%Display the OF as a image on hsv
function display_hsv(Vx,Vy)
    figure
    ang = atan2(Vy, Vx) + pi;
    H = ang * (180/pi/2);
    S = ones(size(Vx)).*255;
    value = sqrt(Vx .^ 2 + Vy .^ 2);
    V = bsxfun(@min,value .* 4,255);
    img_HSV = cat(3, H, S, V);
    imshow(hsv2rgb(img_HSV));
end





