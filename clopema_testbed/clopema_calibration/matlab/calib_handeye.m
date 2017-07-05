function res = calib_handeye(images, camera, conf)
%CALIB_HANDEYE
%
% SYNOPSIS
%   res = handeye(conf, camera)
%
% INPUT
%   conf.rba_binary
%   
%
% OUTPUT
%   res = struct()
%   res.HEC             Hand-eye transformation (4x4)
%   res.WBC             Word-base transformation (4x4)
%   res.scale           Scale between calibre units and robot units

% Jan Heller
% Libor Wagner on April 30, 2013

    % Remove inalid poses
    images = images(logical(cat(2,images.valid)));


    N = length(images);
    M = length(images(1).x);

    % Prepare RBA input
    exp = struct();
    exp.K = camera.K;
    exp.dist = camera.dist;

    exp.ipath = {images.img_path};
    exp.calib = [images(1).ind; zeros(1,M)];
    exp.u = zeros(N,2,M);
    exp.u_index = zeros(N,M);
    for i = 1:N
        exp.u(i,:,:) = images(i).x;
        exp.u_index(i,:) = ~any(isnan(images(i).x),1);
        exp.Rb_abs{i} = images(i).T(1:3,1:3);
        exp.tb_abs{i} = images(i).T(1:3,4);
    end

    rperm = randperm(N);
    exp.xvalid_cameras = rperm(1:floor(N/4));

    exp.calib_report = [conf.output_tpl '/handeye_report'];
    mkdir(exp.calib_report);

    % Get the RBA directory
    LD_PATH = ['LD_LIBRARY_PATH=' conf.ld_path];
    exp.rba_command = [LD_PATH ' ' conf.rba_binary];
    exp.bacalib_type = 5;
    exp.use_images = 1;
    exp.camera_ecoef = 40;
    exp.robot_ecoef = 40;
    exp.calib_camera = 1;

    % Call RBA
    res_rba = mrba(exp);

    % Hand-Eye
    HEC = eye(4,4);
    HEC(1:3,1:3) = res_rba.Rx_rba;
    HEC(1:3,4) = res_rba.tx_rba;

    % World-Base
    WBC = eye(4,4);
    WBC(1:3,1:3) = res_rba.Rz_rba;
    WBC(1:3,4) = res_rba.tz_rba;

    % Scale
    scale = res_rba.scale_rba;

    res = struct();
    res.HEC = HEC;
    res.WBC = WBC;
    res.scale = scale;

end

