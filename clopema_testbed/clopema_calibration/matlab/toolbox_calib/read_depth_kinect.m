% Reads Kinect depth image in the OpenCV yml format

function depth_img = read_depth_kinect(path_raw)
        fid = fopen(path_raw);
        fseek(fid, 124, 'bof');
        depth_data = fscanf(fid, ['%u','., ']);
        fclose(fid);
        depth_img = double(reshape(depth_data, 640,480))';
        depth_img(depth_img == 2047) = nan; % remove 2047 - empty values
end