function exp = mrba(exp)

%tmp = '/tmp/tmpram/';
tmp = '';

% Required fields ---------------------------------------------------------

use_joints = 0;

if (~isfield(exp, 'Rb_abs'))
    if (~isfield(exp, 'J'))
        error('fields Rb_abs/tb_abs of J required');
    else
        if (~isfield(exp, 'robotname'))
            error('field robot required while J is set');
        end
        if (~isfield(exp, 'robotfunc'))
            error('field robotfunc required while J is set');
        end
        use_joints = 1;
    end
end

if (isfield(exp, 'use_joints'))
    if (exp.use_joints == 1)
        if (~isfield(exp, 'J'))
            error('field J required while use_joints==1');
        end
        use_joints = 1;
    end
end

use_images = 0;
if (isfield(exp, 'use_images'))
    if (exp.use_images == 1)
        if (~isfield(exp, 'ipath'))
            error('field ipath required while use_images==1');
        end
        use_images = 1;
    end
end
        
if (~isfield(exp, 'tb_abs') && isfield(exp, 'Rb_abs'))
    error('field tb_abs required while Rb_abs is set');
end

if (~isfield(exp, 'calib'))
    error('field calib required');
end

if (~isfield(exp, 'u'))
    error('field u required');
end


if (isfield(exp, 'Rx_0'))
    if (~isfield(exp, 'tx_0'))
        error('field tx_0 required if Rx_0 is set');
    end
end
    
if (isfield(exp, 'Rz_0'))
    if (~isfield(exp, 'tz_0'))
        error('field tz_0 required if Rz_0 is set');
    end
end


if (isfield(exp, 'use_cameras'))
    if (~isfield(exp, 'Ra_abs'))
        error('field Ra_abs required while use_cameras is set');
    end
    if (~isfield(exp, 'ta_abs'))
        error('field ta_abs required while use_cameras is set');
    end
end

if (isfield(exp, 'K'))
    if (~isfield(exp, 'ipath'))
        if(~isfield(exp, 'iwidth') || ~isfield(exp, 'iheight'))
            error('fields iwidth/iheight required while K is set and ipath is not set');
        end
    end
    if (~isfield(exp, 'dist'))
        error('field dist required while K is set');
    end
end

% Init --------------------------------------------------------------------

if (~isfield(exp, 'rba_command'))
    [dummy,host]=system('hostname');
    if (strcmp(host(1:end-1),'frost'))
        rba_command = 'LD_LIBRARY_PATH=/opt/opencv2/lib/:/home/jheller/workspace/rba/lib/ /home/jheller/workspace/rba/Release/rba';
    else
        rba_command = 'LD_LIBRARY_PATH=/opt/opencv2/lib/:/home.dokt/hellej1/workspace/rba/lib/ /home.dokt/hellej1/workspace/rba/Release/rba';
    end
else
    rba_command = exp.rba_command;
end
    
rba_exp = '';
if (use_joints == 0)
    no_poses = numel(exp.Rb_abs);
else
    no_poses = numel(exp.J);
end    

% Robot poses -------------------------------------------------------------

rppath = [tmp tempname '_robot_poses'];
fid = fopen(rppath, 'w');

fprintf(fid, '%d\n', no_poses);

for i=1:no_poses
    if (use_joints == 0)
        R = exp.Rb_abs{i};
        t = exp.tb_abs{i};

        if (use_images == 1)
            fprintf(fid, '1\n');
            fprintf(fid, '%s\n', exp.ipath{i});
        else
            fprintf(fid, '2\n');
        end
            
        fprintf(fid, '%.20e %.20e %.20e\n%.20e %.20e %.20e\n%.20e %.20e %.20e\n', R(1,1), R(1,2),R(1,3),R(2,1),R(2,2),R(2,3), R(3,1),R(3,2), R(3,3)); 
        fprintf(fid, '%.20e %.20e %.20e\n', t(1), t(2), t(3));
    else
        if (use_images == 1)
            fprintf(fid, '3\n');
            fprintf(fid, '%s\n', exp.ipath{i});
        else
            fprintf(fid, '4\n');
        end
        
        fprintf(fid, '%s\n', exp.robotname);
                
        for j = 1:numel(exp.J{i})
            fprintf(fid, '%.20e ', exp.J{i}(j));
        end
        fprintf(fid, '\n');
    end
end

fclose(fid);

if (use_joints == 1)
    rba_exp = [rba_exp ' --robot_func=' exp.robotfunc];
end

% Calib dev ---------------------------------------------------------------

cdpath = [tmp tempname '_calib_dev'];
fid = fopen(cdpath, 'w');

if (~isfield(exp, 'K'))
    % No internal calibration -- vectors are directions
    no_points = size(exp.calib, 2);
    fprintf(fid, '%d %d 1\n', no_poses, no_points);
    
    for j=1:no_points
        p = exp.calib(:, j);
    	fprintf(fid, '%.20e %.20e %.20e %d ', p(1), p(2), p(3), no_poses);

        for i=1:no_poses
            p = exp.u{i}(:,j);
            fprintf(fid, '%d %.20e %.20e %.20e ', i-1, p(1), p(2), p(3));
        end    
        fprintf(fid, '\n');
    end
else
    % K is present -- vectors are pixels
    
    no_points = size(exp.calib, 2);
    nop = numel(find(sum(exp.u_index)>0));
    
    if (isfield(exp, 'u_index'))
        fprintf(fid, '%d %d 2\n', no_poses, nop);
    else
        fprintf(fid, '%d %d 2\n', no_poses, no_points);
    end
    
    R = exp.K;
    fprintf(fid, '%.20e %.20e %.20e\n%.20e %.20e %.20e\n%.20e %.20e %.20e\n', R(1,1), R(1,2),R(1,3),R(2,1),R(2,2),R(2,3), R(3,1),R(3,2), R(3,3)); 
    for j = 1:numel(exp.dist)
        fprintf(fid, '%.20e ', exp.dist(j));
    end
    fprintf(fid, '\n');
    for j = 1:no_poses
        if (use_images == 1)
            fprintf(fid, '%s\n', exp.ipath{j});
        else
            fprintf(fid, '@ %d %d\n', exp.iwidth, exp.iheight);            
        end
    end
    
    if (isfield(exp, 'u_index'))
        for j=1:no_points
            
            ppos = exp.u_index(:,j);
            pos = find(ppos > 0);
            
            if (numel(pos > 0))
                p = exp.calib(:, j);            
                fprintf(fid, '%.20e %.20e %.20e %d ', p(1), p(2), p(3), numel(pos));

            for i=1:numel(pos)
                p = exp.u(pos(i), :, j);
                idx = pos(i) - 1;
                fprintf(fid, '%d %.20e %.20e ', idx, p(1), p(2));
            end    
            fprintf(fid, '\n');
            end
        end
    else
        for j=1:no_points
            p = exp.calib(:, j);
            fprintf(fid, '%.20e %.20e %.20e %d ', p(1), p(2), p(3), no_poses);

            for i=1:no_poses
                p = exp.u{i}(:,j);
                fprintf(fid, '%d %.20e %.20e ', i-1, p(1), p(2));
            end    
            fprintf(fid, '\n');
        end        
    end
end
    
fclose(fid);

% Camera poses ------------------------------------------------------------

cppath = '';

if (isfield(exp, 'use_cameras'))
  if (exp.use_cameras == 1)
    cppath = [tmp tempname '_camera_poses'];
    fid = fopen(cppath, 'w');

    fprintf(fid, '%d\n', no_poses);
    for i=1:no_poses
        R = exp.Ra_abs{i};
        t = exp.ta_abs{i};
        fprintf(fid, '2\n');
        fprintf(fid, '%.20e %.20e %.20e\n%.20e %.20e %.20e\n%.20e %.20e %.20e\n', R(1,1), R(1,2),R(1,3),R(2,1),R(2,2),R(2,3), R(3,1),R(3,2), R(3,3)); 
        fprintf(fid, '%.20e %.20e %.20e\n', t(1), t(2), t(3));
    end   
    
    fclose(fid);
    
    rba_exp = [rba_exp ' --camera_poses=' cppath];
  end
end

if (isfield(exp,'adjust_cameras'))
    if (exp.adjust_cameras == 1)
        rba_exp = [rba_exp ' --adjust_cameras '];
    end
end

% Camera poses output -----------------------------------------------------

copath = '';

if (isfield(exp, 'cameras_results'))
    if (exp.cameras_results == 1)
        copath = [tmp tempname '_cameras_results'];
        rba_exp = [rba_exp ' --cameras_results=' copath];
    end 
end

% Internal Camera Calibration ---------------------------------------------

if (isfield(exp,'calib_camera'))
    if (exp.calib_camera == 1)
        rba_exp = [rba_exp ' --calib_camera '];
    end
end

% Initial HEC calibration -------------------------------------------------

capath = '';
if (isfield(exp, 'Rx_0') || isfield(exp, 'Rz_0') || isfield(exp, 'scale_0'))
    capath = [tmp tempname '_calib'];
    fid = fopen(capath, 'w');

    % HEC init
    if (isfield(exp, 'Rx_0') && isfield(exp, 'tx_0'))
        if ((numel(exp.Rx_0) ~= 0) || (numel(exp.tx_0) ~= 0))
            fprintf(fid, '1\n');
            R = exp.Rx_0;
            t = exp.tx_0;
            fprintf(fid, '%.20e %.20e %.20e\n%.20e %.20e %.20e\n%.20e %.20e %.20e\n', R(1,1), R(1,2),R(1,3),R(2,1),R(2,2),R(2,3), R(3,1),R(3,2), R(3,3)); 
            fprintf(fid, '%.20e %.20e %.20e\n', t(1), t(2), t(3));
        else
            fprintf(fid, '0\n');
        end
    else
        fprintf(fid, '0\n');
    end        
        
    % WBC init
    if (isfield(exp, 'Rz_0') && isfield(exp, 'tz_0'))
        if ((numel(exp.Rz_0) ~= 0) || (numel(exp.tz_0) ~= 0))
            fprintf(fid, '1\n');
            R = exp.Rz_0;
            t = exp.tz_0;
            fprintf(fid, '%.20e %.20e %.20e\n%.20e %.20e %.20e\n%.20e %.20e %.20e\n', R(1,1), R(1,2),R(1,3),R(2,1),R(2,2),R(2,3), R(3,1),R(3,2), R(3,3)); 
            fprintf(fid, '%.20e %.20e %.20e\n', t(1), t(2), t(3));
        else
            fprintf(fid, '0\n');
        end
    else
        fprintf(fid, '0\n');
    end
       
    % Scale init set to identity - TODO can be changed

    if (isfield(exp, 'scale_0'))
        fprintf(fid, '1\n');
        fprintf(fid, '%.20e\n', exp.scale_0);
    else
        fprintf(fid, '0\n');
    end

    fclose(fid);
    
    rba_exp = [rba_exp ' --calib_txt=' capath];
end

% Bundle Adjustment options -----------------------------------------------

if (isfield(exp, 'bacalib_type'))
    rba_exp = [rba_exp ' --bacalib_type=' sprintf('%d', exp.bacalib_type)]; 
end

% Non Bundle Adjustment options -------------------------------------------

if (isfield(exp, 'pr6hec'))
    if (exp.pr6hec == 1)
        rba_exp = [rba_exp ' --pr6hec ']; 
    end
end

% Robot Params ------------------------------------------------------------

if (isfield(exp, 'robot_pmask'))
    pmask = '';
    for i = 1:numel(exp.robot_pmask)
        if (exp.robot_pmask(i) == 0)
            pmask = [pmask '0'];
        else
            pmask = [pmask '1'];
        end
    end
    rba_exp = [rba_exp ' --robot_pmask=' pmask]; 
end

if (isfield(exp, 'robot_pinit'))
    pinit = '"';
    for i = 1:numel(exp.robot_pinit)
        pinit = [pinit sprintf('%.20e', exp.robot_pinit(i))];
        if (i ~= numel(ex.robot_pinit))
            pinit = [pinit ', '];
        else
            pinit = [pinit '"'];
        end
    end
    rba_exp = [rba_exp ' --robot_pinit=' pinit]; 
end

% Calib report ------------------------------------------------------------

if (isfield(exp, 'calib_report'))
    if (numel(exp.calib_report) > 0)
        rba_exp = [rba_exp ' --calib_report="' exp.calib_report '"']; 
    end
end

if (isfield(exp, 'camera_ecoef'))
    if (exp.camera_ecoef > 0)
        rba_exp = [rba_exp ' --camera_ecoef=' sprintf('%d', exp.camera_ecoef)];
    end
end

if (isfield(exp, 'robot_ecoef'))
    if (exp.robot_ecoef > 0)
        rba_exp = [rba_exp ' --robot_ecoef=' sprintf('%d', exp.robot_ecoef)];
    end
end

% Cross-validation --------------------------------------------------------

if (isfield(exp, 'xvalid_cameras'))
    xc = '"';
    for i = 1:numel(exp.xvalid_cameras)
        xc = [xc sprintf('%d,', exp.xvalid_cameras(i)-1)];  
    end
    xc(end) = '"';
    rba_exp = [rba_exp ' --xvalid_cameras=' xc];
end

% Run RBA -----------------------------------------------------------------

respath = [tmp tempname '_results'];

rba_exp = [rba_exp ' ' '--robot_poses=' rppath ' ' '--calibdev_txt=' cdpath ' ' '--calib_results=' respath];
rba_command = [rba_command ' ' rba_exp];

%disp(rba_command);
[status,result] = system(rba_command);

if (isfield(exp, 'save_report'))
    if (exp.save_report == 1)
        exp.result_rba = result;
    end
end

results = [];

if (exist(respath, 'file'))

    results = load(respath);

    exp.Rx_rba = results(1:3,1:3);
    exp.tx_rba = results(1:3,4);

    exp.Rz_rba = results(5:7,1:3);
    exp.tz_rba = results(5:7,4);

    exp.scale_rba = results(9,1);

    if (numel(results) > 36)
        rparams = results(10:end,:);
        rparams = rparams';
        for i = 1:numel(rparams);
            if (~isnan(rparams(i)))
                exp.rparams_rba(i) = rparams(i);
            end
        end
    end

    if(~strcmp(copath, ''))
        cameras = load(copath);
        for i = 1:(size(cameras, 1)/4)
            pos = 4 * (i - 1) + 1; 
            exp.Ra_abs_rba{i} = cameras(pos:(pos+2),:);
            exp.ta_abs_rba{i} = cameras(pos+3,:)';
        end
    end
end

% Clean up ----------------------------------------------------------------

if (exist(rppath, 'file'))
    delete(rppath);
end

if (exist(cdpath, 'file'))
    delete(cdpath);
end

if (exist(respath, 'file'))
    delete(respath);
end

if (exist(cppath, 'file'))
    delete(cppath);
end

if (exist(capath, 'file'))
    delete(capath);
end

if (exist(copath, 'file'))
    delete(copath);
end

if (numel(results) == 0)
    error(result);
end

end
