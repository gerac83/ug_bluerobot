#!/usr/bin/env bash


eval "$(rosrun clopema_calibration docopts -h - : "$@" <<EOF
Usage: run_calib_camera.sh -i INPUT_DIR [-c NAME] [options]
       run_calib_camera.sh -h | --help

Options:
    -h, --help                          Show help message.
    -i INPUT_DIR, --input INPUT_DIR     Input directory containing images and
                                        other calibration files produced by the
                                        camera_calib_capture.py.
    -o OUTPUT_DIR, --output OUTPUT_DIR  Output directory for reports and stuff
                                        if not given the input directoru is
                                        suffixed with _sub and used as output
                                        directory.
    -c NAME, --commit NAME              Add the calibration into the repository
                                        under given name.
    --hand-eye LINK                     Perform handeye calibration for the
                                        following link. Use xtionX_link_ee.
Advanced Options:
    --gdf PATH                          Path to gauge defnition file.
    --alg PATH                          Path to dotmatrix algorithm.
EOF
)"

# Find clopema_calibration package
CLOPEMA_CALIBRATION=`rospack find clopema_calibration`
CLOPEMA_DESCRIPTION=`rospack find clopema_description`

# Default algorithm
if [[ -z "$alg" ]]; then
    alg=$CLOPEMA_CALIBRATION/config/algorithm_1.dma
fi

# Default gauge description file
if [[ -z "$gdf" ]]; then
    gdf=$CLOPEMA_CALIBRATION/config/gauge30x30.gdf
fi

# Default output directory
if [[ -z "$output" ]]; then
    output=$input\_out
fi

# Make the output dire if missing
mkdir $output

# Run calibration in matlab
matlab -nodisplay -nojvm << EOF
addpath(genpath('$CLOPEMA_CALIBRATION/matlab'));

conf = struct();

% Input configuration
conf.input_dir = '$input';
conf.input_ext = {'png', 'txt'};

% Dotmatrix and HandEye libraries path
conf.ld_path = '$CLOPEMA_CALIBRATION/bin3rd/lib:/usr/lib/lapack:/opt/ros/hydro/lib';

% Dotmatrix configuration
conf.dm_binary = '$CLOPEMA_CALIBRATION/bin3rd/dotmatrix';
conf.dm_algorithm = '$alg';
conf.dm_gauge = '$gdf';

% Common options
conf.verbose = 0;
conf.show = 0;

% Camera calibration
conf.output_tpl = '$output';
conf.cc_scale = 0.039;

% RBA configuration
conf.rba_binary = '$CLOPEMA_CALIBRATION/bin3rd/rba';

% Search input directory for images
disp('');
conf.calib_dir = calib_dir(conf);

% Check whether we have RGB images
if ~isfield(conf.calib_dir, 'RGB'),
    error(['No RGB images foud in the calibration directory: ', conf.input_dir]);
end

conf.images = calib_dotmatrix({conf.calib_dir.RGB}, conf);

% Read the transformations
for i = 1:length(conf.calib_dir)
    conf.images(i).T = readtf(conf.calib_dir(i).T);
end

conf.camera = calib_camera(conf.images, conf);
im = imread(conf.calib_dir(1).RGB);
conf.camera.width = size(im,2);
conf.camera.height = size(im,1);

disp('');
disp('========================================================================');
disp('========================================================================');

export_camera(conf.camera, '$output/rgb_camera.yaml');

if length('$commit') > 0
    conf.camera.name = '$commit';
    export_camera(conf.camera, ['$CLOPEMA_DESCRIPTION/calibration_$CLOPEMA_PARTNER/$commit' '_camera.yaml']);
end

if length('$hand_eye') > 0
    hand_eye = calib_handeye(conf.images, conf.camera, conf);
    export_link('$output/rgb_handeye.calib', '$hand_eye', hand_eye.HEC);
    if length('$commit') > 0
        export_link(['$CLOPEMA_DESCRIPTION/calibration_$CLOPEMA_PARTNER/$commit' '_handeye.calib'], '$hand_eye', hand_eye.HEC);
    end
end

EOF
