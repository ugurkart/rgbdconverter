function [rgb_frames, depth_frames, init_bb] = load_princeton_video(video_path, video_name)

% These are the sequences that had issues with registration and
% synchronization in the original PTB dataset
reg_sync = {'basketball1', 'cup_book', 'library2.1_occ', 'new_student_center1', 'new_student_center2', 'new_student_center3', 'studentcenter3.1', 'walking_no_occ'};
only_sync = {'bear_back', 'cafe_occ1', 'cf_occ2', 'cf_occ3', 'dog_occ_3', 'hand_no_occ', 'new_ex_occ1', 'new_ex_occ6', 'new_ex_occ7.1', 'rose1.2', 'static_sign1', 'toy_car_occ', 'wr_no'};

reg_sync_flag = false;
for f=1:length(reg_sync)
    if(strcmp((reg_sync{f}), video_name))
        reg_sync_flag = true;
        break;
    end
end

only_sync_flag = false;
for f=1:length(only_sync)
    if(strcmp((only_sync{f}), video_name))
        only_sync_flag = true;
        break;
    end
end

ground_truth = dlmread([video_path '/init.txt']);

init_bb = ground_truth(1,:);
init_bb = init_bb + 1;

load([video_path 'frames']);

if(reg_sync_flag || only_sync_flag)
    
    video_dir = dir([video_path '/corrected_depth']); % This refers to the depth images provided by Bibi et al. CVPR 2016
    
    depth_img_names = cell(max(size(video_dir))-2, 1);
    depth_img_names_indices = cell(max(size(video_dir))-2, 1);
    video_dir_cell = struct2cell(video_dir);
    
    for d=3:max(size(video_dir))
        depth_img_names{d-2} = video_dir_cell{1, d};
        C = strsplit(depth_img_names{d-2}, '-');
        depth_img_names_indices{d-2} = C{1,3};
    end
    
    [sorted_depth_img_names, index] = sort_nat(depth_img_names);
    [sorted_depth_img_names_indices, index] = sort_nat(depth_img_names_indices);
    
end

if((only_sync_flag || reg_sync_flag)  == true)
    load([video_path 'FrameID_sync']);
end

num_images = frames.length;
image_names = cell(num_images, 1);
depth_frames = cell(num_images, 1);
for j=1:num_images
    image_names{j} = fullfile(video_path,sprintf('rgb/r-%d-%d.png', frames.imageTimestamp(j), frames.imageFrameID(j)));
    if(reg_sync_flag == true)
        depth_img_index = find(ismember(sorted_depth_img_names_indices, [num2str(FrameID_sync(j)) '.png']));
        depth_frames{j} = fullfile(video_path,sprintf('corrected_depth/%s', sorted_depth_img_names{depth_img_index}));
    elseif(only_sync_flag == true)
        depth_img_index = find(ismember(sorted_depth_img_names_indices, [num2str(FrameID_sync(j)) '.png']));
        depth_frames{j} = fullfile(video_path,sprintf('corrected_depth/%s', sorted_depth_img_names{depth_img_index}));
    else
        depth_frames{j} = fullfile(video_path,sprintf('corrected_depth/d-%d-%d.png', frames.depthTimestamp(j), frames.depthFrameID(j)));
    end
end

[rgb_frames, index] = sort_nat(image_names);

end

