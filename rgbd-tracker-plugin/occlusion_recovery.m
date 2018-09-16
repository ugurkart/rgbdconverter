function [rgbd_plugin, c, bb, resp_score, temp_params] = occlusion_recovery(rgbd_plugin, tracker, rgb_img)

search_region = zeros(747, 747, 3);

if(max(size(rgbd_plugin.consistent_centers)) == rgbd_plugin.successful_frame_limit)
    occlusion_recovery_region_top = uint16(rgbd_plugin.last_known_position - ...
        (rgbd_plugin.num_occluded_frames * rgbd_plugin.consistent_velocity) - rgbd_plugin.last_bb(3:4));
    occlusion_recovery_region_bottom = uint16(rgbd_plugin.last_known_position + ...
        (rgbd_plugin.num_occluded_frames * rgbd_plugin.consistent_velocity) + rgbd_plugin.last_bb(3:4));
else
    occlusion_recovery_region_top = uint16(rgbd_plugin.last_known_position - ...
        (rgbd_plugin.num_occluded_frames * rgbd_plugin.default_velocity) - rgbd_plugin.last_bb(3:4));
    occlusion_recovery_region_bottom = uint16(rgbd_plugin.last_known_position + ...
        (rgbd_plugin.num_occluded_frames * rgbd_plugin.default_velocity) + rgbd_plugin.last_bb(3:4));
end

if(occlusion_recovery_region_top(1, 1) < 1)
    occlusion_recovery_region_top(1, 1) = 1;
end

if(occlusion_recovery_region_top(1, 2) < 1)
    occlusion_recovery_region_top(1, 2) = 1;
end

if(occlusion_recovery_region_bottom(1, 1) > size(rgb_img, 2))
    occlusion_recovery_region_bottom(1, 1) = size(rgb_img, 2);
end

if(occlusion_recovery_region_bottom(1, 2) > size(rgb_img, 1))
    occlusion_recovery_region_bottom(1, 2) = size(rgb_img, 1);
end

if(rgbd_plugin.debug)
    new_img(:,:,:) = rgb_img(occlusion_recovery_region_top(1, 2):occlusion_recovery_region_bottom(1, 2),...
        occlusion_recovery_region_top(1, 1):occlusion_recovery_region_bottom(1, 1), :);
    new_img = imresize(new_img, [size(search_region, 1) size(search_region, 2)]);
    rgbd_plugin.search_region = new_img;
end

[max_response, c, bb, temp_params] = detect_best_response(rgb_img, tracker, occlusion_recovery_region_top, occlusion_recovery_region_bottom);

c = double(c);
rgbd_plugin.current_response = max_response;
region = [c - tracker.currentScaleFactor * tracker.base_target_sz/2, tracker.currentScaleFactor * tracker.base_target_sz];
bb = region;
rgbd_plugin.bb = bb;
resp_score = max_response;

[response_occlusion_state, rgbd_plugin] = analyze_mean_response(rgbd_plugin);

if(response_occlusion_state == false)
    rgbd_plugin.first_iteration = true;
    rgbd_plugin.occlusion = false;  
else
    return;
end

end

