%==========================================================================
% Lane Detection Project
% ECE 415 | Professor Cetin
% Written by: Qudsia Sultana & Shahed Alhanbali
%==========================================================================

%-------------------------Importing the Video File-------------------------
VideoFile = VideoReader('project_video.mp4');

%-------------------Loading Region of Interest Variables-------------------
load('variables', 'c', 'r');

%-----------------Defining Variables for saving Video File-----------------
Output_Video = VideoWriter('Result');
Output_Video.FrameRate = 25;
open(Output_Video);

%% Initializing the loop to take frames one by one
while hasFrame(VideoFile)
    frame = readFrame(VideoFile);    
    frame = imgaussfilt3(frame);

    % Define thresholds for Yellow and White lane masking
    channel1MinY = 130; channel1MaxY = 255;
    channel2MinY = 130; channel2MaxY = 255;
    channel3MinY = 0; channel3MaxY = 130;

    Yellow = ((frame(:,:,1) >= channel1MinY) | (frame(:,:,1) <= channel1MaxY)) & ...
             (frame(:,:,2) >= channel2MinY) & (frame(:,:,2) <= channel2MaxY) & ...
             (frame(:,:,3) >= channel3MinY) & (frame(:,:,3) <= channel3MaxY);

    channel1MinW = 200; channel1MaxW = 255;
    channel2MinW = 200; channel2MaxW = 255;
    channel3MinW = 200; channel3MaxW = 255;

    White = ((frame(:,:,1) >= channel1MinW) | (frame(:,:,1) <= channel1MaxW)) & ...
            (frame(:,:,2) >= channel2MinW) & (frame(:,:,2) <= channel2MaxW) & ...
            (frame(:,:,3) >= channel3MinW) & (frame(:,:,3) <= channel3MaxW);

    % Detecting edges in the image using Canny edge function
    frameW = edge(White, 'canny', 0.2);
    frameY = edge(Yellow, 'canny', 0.2);

    % Neglecting closed edges in smaller areas
    frameY = bwareaopen(frameY, 15);
    frameW = bwareaopen(frameW, 15);

    % Extracting Region of Interest (ROI)
    roiY = roipoly(frameY, r, c);
    roiW = roipoly(frameW, r, c);

    frame_roiY = roiY .* frameY;
    frame_roiW = roiW .* frameW;

    % Applying Hough Transform to detect lane lines
    [H_Y, theta_Y, rho_Y] = hough(frame_roiY);
    [H_W, theta_W, rho_W] = hough(frame_roiW);

    P_Y = houghpeaks(H_Y, 2, 'threshold', 2);
    P_W = houghpeaks(H_W, 2, 'threshold', 2);

    lines_Y = houghlines(frame_roiY, theta_Y, rho_Y, P_Y, 'FillGap', 3000, 'MinLength', 20);
    lines_W = houghlines(frame_roiW, theta_W, rho_W, P_W, 'FillGap', 3000, 'MinLength', 20);

    % Extrapolating and plotting lane lines
    left_plot = [lines_Y(1).point1; lines_Y(1).point2];
    right_plot = [lines_W(1).point1; lines_W(1).point2];

    % Calculate slope and extend lane lines
    slopeL = (left_plot(2,2) - left_plot(1,2)) / (left_plot(2,1) - left_plot(1,1));
    slopeR = (right_plot(2,2) - right_plot(1,2)) / (right_plot(2,1) - right_plot(1,1));

    xLeftY = 1; % x-coordinate for the left edge
    yLeftY = slopeL * (xLeftY - left_plot(1,1)) + left_plot(1,2);
    xRightY = 550; % x-coordinate for the right edge
    yRightY = slopeL * (xRightY - left_plot(2,1)) + left_plot(2,2);

    xLeftW = 750; % x-coordinate for the left edge
    yLeftW = slopeR * (xLeftW - right_plot(1,1)) + right_plot(1,2);
    xRightW = 1300; % x-coordinate for the right edge
    yRightW = slopeR * (xRightW - right_plot(2,1)) + right_plot(2,2);

    %------Making a transparent Trapezoid between 4 points of 2 lines-------
    points = [xLeftY yLeftY; xRightY yRightY; xLeftW yLeftW; xRightW yRightW];
    number = [1 2 3 4];

    %% Plotting the final output
    figure('Name', 'Final Output');
    imshow(frame);
    hold on;
    plot([xLeftY, xRightY], [yLeftY, yRightY], 'LineWidth', 8, 'Color', 'red');
    plot([xLeftW, xRightW], [yLeftW, yRightW], 'LineWidth', 8, 'Color', 'red');
    patch('Faces', number, 'Vertices', points, 'FaceColor', 'green', ...
          'EdgeColor', 'green', 'FaceAlpha', 0.4);
    hold off;

    % Save each frame to the output video
    writeVideo(Output_Video, getframe);
end

% Closing video file
close(Output_Video);