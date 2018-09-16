function [rgbd_plugin] = rgbd_plugin_process_frame(rgb_img, depth_img, c, bb, rgbd_plugin)

if strcmp(rgbd_plugin.seg_colorspace, 'rgb')
    seg_img = rgb_img;
elseif strcmp(rgbd_plugin.seg_colorspace, 'hsv')
    seg_img = rgb2hsv(rgb_img);
    seg_img = seg_img * 255;
else
    error('Unknown colorspace parameter');
end

obj_reg = round([bb(1), bb(2), bb(1) + bb(3), bb(2) + bb(4)]) - [1 1 1 1];

% extract masked patch: mask out parts outside image
[seg_patch, valid_pixels_mask] = get_patch(seg_img, c, rgbd_plugin.currentScaleFactor, rgbd_plugin.template_size);
[depth_seg_patch, depth_valid_pixels_mask] = get_patch(depth_img, c, rgbd_plugin.currentScaleFactor, rgbd_plugin.template_size);

depth_seg_patch = double(depth_seg_patch) .* depth_valid_pixels_mask; % Remove padded pixels

patch_center_x = size(depth_seg_patch, 2) / 2;
patch_center_y = size(depth_seg_patch, 1) / 2;
object_region_x = max(1, patch_center_x - (bb(3) / 2));
object_region_y = max(1, patch_center_y - (bb(4) / 2));

object_region_bottom_x = min(size(depth_seg_patch, 2), object_region_x + bb(3));
object_region_bottom_y = min(size(depth_seg_patch, 1), object_region_y + bb(4));
rgbd_plugin.obj_region = [object_region_x object_region_y bb(3) bb(4)];

rgbd_plugin.target_region = hsv2rgb(seg_patch ./ 255);
if(rgbd_plugin.first_iteration == true) % Reset the depth histograms after occlusion recovery
    
    depth_seg_patch = depth_seg_patch .* depth_valid_pixels_mask; % Remove padded pixels
    
    depth_img_object_region = depth_seg_patch(object_region_y:object_region_bottom_y, object_region_x:object_region_bottom_x); % Extract object region
    
    [foreground_depth_hist, ~] = histcounts(depth_img_object_region, rgbd_plugin.edges); % Get histogram from object region
    
    depth_valid_pixels_mask(object_region_y : object_region_bottom_y, object_region_x :object_region_bottom_x) = -1000; % Mark foreground pixels
    
    background_pixels = find(depth_valid_pixels_mask == 1); % Background pixel indices on the image
    
    [background_depth_hist, ~] = histcounts(depth_seg_patch(background_pixels), rgbd_plugin.edges);
    
    [foreground_depth_hist, background_depth_hist] = process_histograms(foreground_depth_hist, background_depth_hist);
    
    rgbd_plugin.prev_foreground_depth_hist = foreground_depth_hist;
    rgbd_plugin.prev_background_depth_hist = background_depth_hist;
    
    rgbd_plugin.foreground_depth_hist = foreground_depth_hist;
    rgbd_plugin.background_depth_hist = background_depth_hist;
end

tracker_bb_rect = uint16([object_region_y object_region_x object_region_bottom_y object_region_bottom_x]);

[rgbd_plugin.depth_p] = get_depth_priors(depth_seg_patch, rgbd_plugin.foreground_depth_hist, rgbd_plugin.background_depth_hist);

[fg_p, bg_p] = get_location_prior([1 1 size(seg_patch, 2) size(seg_patch, 1)],...
    rgbd_plugin.currentScaleFactor * rgbd_plugin.base_target_sz, [size(seg_patch,2), size(seg_patch, 1)]);

seg_patch = double(seg_patch) .* valid_pixels_mask; % Remove padded pixels

foreground_priors = rgbd_plugin.depth_p .* fg_p;
background_priors = (1.0 - rgbd_plugin.depth_p) .* bg_p;

[~, fg, ~] = mex_segment(seg_patch, rgbd_plugin.hist_fg, rgbd_plugin.hist_bg, rgbd_plugin.nbins, foreground_priors, background_priors);

% cut out regions outside from image
mask = single(fg).*single(valid_pixels_mask);
mask = binarize_softmask(mask);

bb_area = ((object_region_bottom_y - object_region_y) * (object_region_bottom_x - object_region_x)); %region(3) * region(4);

tracker_bb_region = zeros(size(mask));
tracker_bb_region(object_region_y : object_region_bottom_y, object_region_x : object_region_bottom_x) = 1;

mask = mask .* tracker_bb_region;
rgbd_plugin.mask = mask;

non_zero_foreground_indices = find(mask > 0); % Check if we were able to segment enough

if(rgbd_plugin.occlusion == true)
    if((numel(non_zero_foreground_indices) < bb_area * rgbd_plugin.occlusion_threshold))
        rgbd_plugin.occlusion = true;
        return;
    else
        rgbd_plugin.first_iteration = true;
        rgbd_plugin.occlusion = false;
    end
else
    if((numel(non_zero_foreground_indices) < bb_area * rgbd_plugin.occlusion_threshold))
        if(rgbd_plugin.first_iteration == false)
            if(((rgbd_plugin.frame_no - rgbd_plugin.occlusion_recovery_frame_no) > rgbd_plugin.occlusion_frame_recovery_limit)...
                    || (rgbd_plugin.frame_no <= rgbd_plugin.occlusion_frame_recovery_limit))
                rgbd_plugin.last_known_position = c;
                rgbd_plugin.last_bb = bb;
                rgbd_plugin.consistent_velocity = zeros(1, 2);
                rgbd_plugin.num_occluded_frames = 0;
                rgbd_plugin.successful_frame_index = 1;
                rgbd_plugin.consistent_centers = [];
                rgbd_plugin.consistent_centers(rgbd_plugin.successful_frame_index, :) = c;
                rgbd_plugin.successful_frame_index = rgbd_plugin.successful_frame_index + 1;
            end
        end
        rgbd_plugin.occlusion = true;
        
        return;
    end
end

if(rgbd_plugin.debug == false)
    search_region = 0;
    color_p = 0;
    color_priors = 0;
else
    if(rgbd_plugin.occlusion == false)
        search_region = hsv2rgb(seg_patch./255);
    end
    color_p = get_color_priors(seg_patch, rgbd_plugin.hist_fg, rgbd_plugin.hist_bg, rgbd_plugin.nbins, tracker_bb_rect);
    color_p = color_p .* valid_pixels_mask;
    rgbd_plugin.color_p = color_p;
end

rgbd_plugin.first_iteration =  false;

if(rgbd_plugin.successful_frame_index > 1)
    tmp_center_dist = sqrt((c(1) - rgbd_plugin.consistent_centers(rgbd_plugin.successful_frame_index - 1, 1))^2 + ...
        (c(2) - rgbd_plugin.consistent_centers(rgbd_plugin.successful_frame_index - 1, 2))^2);
else
    tmp_center_dist = sqrt((c(1) - rgbd_plugin.consistent_centers(rgbd_plugin.successful_frame_limit, 1))^2 + ...
        (c(2) - rgbd_plugin.consistent_centers(rgbd_plugin.successful_frame_limit, 2))^2);
end

if(tmp_center_dist <= rgbd_plugin.consistency_radius)
    % Count the consistent trackings
    rgbd_plugin.consistent_centers(rgbd_plugin.successful_frame_index, :) = c;
    rgbd_plugin.successful_frame_index = rgbd_plugin.successful_frame_index + 1;
    
    if(rgbd_plugin.successful_frame_index > rgbd_plugin.successful_frame_limit)
        rgbd_plugin.successful_frame_index = 1;
    end
    
else % Reset counter
    rgbd_plugin.successful_frame_index = 1;
    rgbd_plugin.consistent_centers = [];
    rgbd_plugin.consistent_centers(rgbd_plugin.successful_frame_index, :) = c;
    rgbd_plugin.successful_frame_index = rgbd_plugin.successful_frame_index + 1;
end

[foreground_depth_hist, ~] = histcounts(depth_seg_patch(non_zero_foreground_indices), rgbd_plugin.edges); % Get histogram from the segmented region

depth_valid_pixels_mask(object_region_y : object_region_bottom_y, object_region_x : object_region_bottom_x) = -1000; % Mark foreground pixels

background_pixels = find(depth_valid_pixels_mask == 1); % Background pixel indices on the image

[background_depth_hist, ~] = histcounts(depth_seg_patch(background_pixels), rgbd_plugin.edges);

[foreground_depth_hist, background_depth_hist] = process_histograms(foreground_depth_hist, background_depth_hist);

rgbd_plugin.prev_foreground_depth_hist = rgbd_plugin.foreground_depth_hist;
rgbd_plugin.prev_background_depth_hist = rgbd_plugin.background_depth_hist;

max_foreground_depth_hist = max(max(rgbd_plugin.foreground_depth_hist));
max_foreground_depth_hist_index = find(rgbd_plugin.foreground_depth_hist == max_foreground_depth_hist);

max_gauss_index = find(rgbd_plugin.gauss_win == max(max(rgbd_plugin.gauss_win)));
max_gauss_index = max_gauss_index(1);

shift_offset = numel(rgbd_plugin.foreground_depth_hist) - max_gauss_index + max_foreground_depth_hist_index;

current_gauss_win = circshift(rgbd_plugin.gauss_win, shift_offset);

rgbd_plugin.foreground_depth_hist = (1-rgbd_plugin.depth_hist_lr)*rgbd_plugin.foreground_depth_hist + ((rgbd_plugin.depth_hist_lr*foreground_depth_hist) .* current_gauss_win');
rgbd_plugin.background_depth_hist = (1-rgbd_plugin.depth_hist_lr)*rgbd_plugin.background_depth_hist + rgbd_plugin.depth_hist_lr*background_depth_hist;

int_mask = uint8(mask);
seg_patch_obj_reg =  [object_region_x object_region_y object_region_bottom_x object_region_bottom_y];
hist_fg = mex_extractforeground(seg_patch, seg_patch_obj_reg, rgbd_plugin.nbins, int_mask, 1);
hist_bg = mex_extractbackground(seg_img, obj_reg, rgbd_plugin.nbins);
rgbd_plugin.hist_fg = (1-rgbd_plugin.hist_lr) * rgbd_plugin.hist_fg + rgbd_plugin.hist_lr * hist_fg;
rgbd_plugin.hist_bg = (1-rgbd_plugin.hist_lr) * rgbd_plugin.hist_bg + rgbd_plugin.hist_lr * hist_bg;
end

