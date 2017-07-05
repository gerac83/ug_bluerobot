function res = calib_depth(conf)
%CALIB_DEPTH
%
% SYNOPSIS
%   res = calib_depth(conf)
%
% INPUT
%   conf = struct()
%   conf.camera_ir
%   conf.pose = struct()
%   conf.pose(i).paht_dis
%   conf.pose(i).x
%   conf.output
%
% OUTPUT
%   res = struct()
%

% Jan Smisek
% Libor Wagner on May  3, 2013

cal_IR = load(conf.camera_ir.calib);
n_ima = length(conf.pose);
col = 'brgkcmyr'; % plot colors

b = 75/1000; %  kinect baseline [m]
f = mean(cal_IR.fc); % kinect focal lenght [px]
pix_size = 0.0104/1000; % pixel size [mm]
f_mm = f*pix_size/1000; % focal lenght [mm]
x_off = -3; y_off = -3; % Xd = Xir + x_off;
active_images = logical(cat(2, conf.pose.valid_ir));


for index = 1:length(conf.pose)
    if active_images(index)
        path_raw = conf.pose(index).path_dis;
        disp(['Processing: ' path_raw]);
        cp = load(path_raw);
        im = double(cp.image); im(im == 2047) = NaN;
        depth_rect(:,:,index) = rect(double(im),eye(3),cal_IR.fc,cal_IR.cc,cal_IR.kc,cal_IR.alpha_c,cal_IR.KK); %rectify the depth image
    end
end

for i = 1:n_ima,
    if active_images(i)

        eval(['x_kk = cal_IR.x_' num2str(i) ';']);
        eval(['XX_kk = cal_IR.X_' num2str(i) ';']);
        eval(['R_kk = cal_IR.Rc_' num2str(i) ';']);
        eval(['Tc_kk = cal_IR.Tc_' num2str(i) ';']);

        % rectify control points positions with known IR camera distortions
        x_n = normalize_pixel(x_kk,cal_IR.fc,cal_IR.cc,cal_IR.kc,cal_IR.alpha_c);
        x_kk = [x_n(1,:)*cal_IR.fc(1) + cal_IR.cc(1); x_n(2,:)*cal_IR.fc(2) + cal_IR.cc(2)];

        % inverse disparities returned by Kinect at the control points
        d_extracted = interp2(depth_rect(:,:,i), x_kk(1,:)+x_off+1, x_kk(2,:)+y_off+1);
        d_kinect{i} = d_extracted(:);

        % distance to photogrammetricaly reconstructed points
        Y_ir{i} = R_kk * XX_kk + Tc_kk * ones(1,length(XX_kk));
        d_ok{i} = sqrt(sum(Y_ir{i}.^2));
    end
end

d_kinect  = d_kinect(~cellfun('isempty', d_kinect));
d_kin_bar = cat(1,d_kinect{:})';

Y_ir_bar  = Y_ir(~cellfun('isempty', Y_ir));
Y_ir_bar  = cat(2,Y_ir_bar{:});

d_bar = (b*f_mm)./(Y_ir_bar(3,:)-f_mm);

assert(size(d_bar, 2) == size(d_kin_bar, 2));

disparity_p = polyfit(d_kin_bar(~isnan(d_kin_bar)),d_bar(~isnan(d_kin_bar)),1)
save([conf.output '/depth_model.txt'],'-ascii','disparity_p');

disp('Depth model calibration - done and saved')


%% Depth calibration accuracy analysis

for i = 1:n_ima,

    if active_images(i)
        eval(['x_kk = cal_IR.x_' num2str(i) ';']);


        % Undistort x_kk (convert to image plane, and back to pixels)
        x_n = normalize_pixel(x_kk,cal_IR.fc,cal_IR.cc,cal_IR.kc,cal_IR.alpha_c);
        x_kk = [x_n(1,:)*cal_IR.fc(1) + cal_IR.cc(1); x_n(2,:)*cal_IR.fc(2) + cal_IR.cc(2)];


        %for whole disparity image calculate the depths
        d = ((b*f_mm)./(polyval(disparity_p,depth_rect(:,:,i))))+f_mm;

        % extract depths at control points, if Nan -> interpolate
        d_bar = interp2(d, x_kk(1,:)+x_off+1, x_kk(2,:)+y_off+1);
        if any(any(isnan(d_bar)))
            disp(['Found NaN in the image. Filing by interpolation. In image num:' num2str(i)]);
            d = inpaint_nans(d); %in case of Nan, fill the gaps
            d_bar = interp2(d, x_kk(1,:)+x_off+1, x_kk(2,:)+y_off+1);
        end
        dist = d_bar(:);

        g = cal_IR.inv_KK * a2h(x_kk); % reproject rays
        Y_ref{i} = [dist'.*g(1,:); dist'.*g(2,:); dist'.*g(3,:)]; % form the point cloud
    end
end

% calculate residues
for i = 1:n_ima
    if active_images(i)
         Yir = Y_ir{i};
         Yref = Y_ref{i};
         Rvec = Yir - Yref;
         s = sign(dot([0,0,1]'*ones(1,length(Rvec)), Rvec));
         resida{i} = sqrt(sum(Rvec.^2,1)).*s;
    end
end


% Plot vec
S = 10;
figure(); hold on;
for i = 1:n_ima
    if active_images(i)
         Yir = Y_ir{i};
         Yref = Y_ref{i};
         Rvec = Yir - Yref;
         s = sign(dot([0,0,1]'*ones(1,length(Rvec)), Rvec));

         quiver3a(Yref([1,3,2],:)*1000, Yir([1,3,2],:)*1000, S, [col(rem(i-1,8)+1)]);
    end
end
showCamera(eye(3), [0;0;0], 'IR', 1); % draw cameras
view(3);
hold off;

disp('Cal.     Mean          Std          Max error')
for i = 1:n_ima
    if active_images(i)
        fprintf(1,'%3d: %12.5f  %12.5f  %12.5f\n', i, mean(resida{i})*1000, std(resida{i})*1000, max(resida{i})*1000);
    end
end

figH = figure(); %export a hidden figure
hold on; grid on;

%calculate local statistics
d_ok_bar = cat(2,d_ok{~cellfun('isempty', d_ok)});
resida_bar = cat(2,resida{active_images});
bins = min(d_ok_bar(:)):(max(d_ok_bar(:))-min(d_ok_bar(:)))/50:max(d_ok_bar(:));
clear m_b
for i = 1:size(bins,2)-1
    m_b(i) = std(resida_bar((d_ok_bar>bins(i) & d_ok_bar<bins(i+1))));
end

%PLOT residues between estimated position and ground truth
for i = 1:n_ima,
    if active_images(i),
        d_ok_bar = d_ok{i};
        resida_bar = resida{i};
        plot(d_ok_bar(1,:)*1000,resida_bar*1000,['.' col(rem(i-1,8)+1)]);
    end
end
plot(bins(2:end)*1000,m_b*1000,'r.-','LineWidth',3)
resida_bar = cat(2,resida{active_images});

xlabel('True distance to the target point [mm]');
ylabel('Error - geometrical distance from ground thruth [mm]');
title(['Error between Kinect and Camera reconstructed points (mean: ' num2str(mean(resida_bar(:))*1000) ' std: ' num2str(std(resida_bar(:))*1000) ')']);
print(figH,'-depsc',[conf.output '/kin_residues.eps']); % save results



resida_bar = cat(2,resida{active_images});
figH = figure('visible','off'); %export a hidden figure
[resida_n, resida_bin] = hist(resida_bar(:)*1000, 0:5:50);
resida_n = resida_n(:)./sum(resida_n(:));
%bar(resida_bin',resida_n','hista')
stairs(resida_bin',resida_n','LineWidth',2)
grid
% axis([0 15 0 0.25])
ylabel('Normalized occurence');
xlabel('Error - geometrical distance from ground thruth [mm]');
title(['Error between Kinect and Camera reconstructed points (mean: ' num2str(mean(resida_bar(:))) ' std: ' num2str(std(resida_bar(:))) ')']);
print(figH,'-depsc',[conf.output '/kin_residues_hist.eps']); % save results


res = struct();
res.disp = disparity_p;

end
