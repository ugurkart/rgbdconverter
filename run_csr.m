clc
close all
clear all
warning off all
dbstop if error

visualize = true;
debug = true;

tracker_path = '/home/ugurkart/Codebase/csr_dcf_rgbd_paper/';

root_path = [tracker_path '/EvaluationSetDemo/']; 
all_video_dir = dir(root_path);
num_videos = max(size(all_video_dir));

for i=3:num_videos
    
    video_name = all_video_dir(i).name
    video_path = [root_path video_name '/'];
    
    output_folder = [tracker_path '/eccvw_results/'];
    mkdir(output_folder);
    
    file_name = [output_folder video_name '.txt'];
    fileID = fopen(file_name, 'w');
    
    [rgb_frames, depth_frames, init_rect] = load_princeton_video(video_path, video_name);
    
    fprintf(fileID,'%f,%f,%f,%f\n', init_rect(1), init_rect(2), init_rect(1) + init_rect(3), init_rect(2) + init_rect(4));
    
    init_rgb_img = imread(rgb_frames{1});
    init_depth_img = double(imread(depth_frames{1}));
    
    [rgbd_plugin] = rgbd_plugin_init(init_rgb_img, init_depth_img, init_rect, visualize, debug);
    
    tracker = tracker_init(init_rgb_img, init_rect, rgbd_plugin.mask);
        
    if(rgbd_plugin.visualize)
        rgbd_plugin = visualize_all(rgbd_plugin, init_rgb_img, init_rect);
    end
    
    for f=2:length(rgb_frames)
        rgbd_plugin.frame_no = f;
        img = imread(rgb_frames{f});
        depth_img = double(imread(depth_frames{f}));
        
        if(rgbd_plugin.occlusion == false)
            [bb, resp_score, temp_params] = tracker_process_frame(tracker, tracker.c, img, false);
            temp_params.c = single(temp_params.c);
            temp_params.bb = single(temp_params.bb);
            rgbd_plugin.c = temp_params.c;
            rgbd_plugin.bb = temp_params.bb;
            rgbd_plugin.currentScaleFactor = temp_params.currentScaleFactor;
            rgbd_plugin.response(rgbd_plugin.frame_counter) = resp_score;
            
            tracker.c = temp_params.c;
            tracker.bb = temp_params.bb;
            tracker.currentScaleFactor = temp_params.currentScaleFactor;
            
            if(mod(rgbd_plugin.frame_counter, rgbd_plugin.history) == 0)
                rgbd_plugin.frame_counter = 1;
            else
                rgbd_plugin.frame_counter = rgbd_plugin.frame_counter + 1;
            end
            
            [rgbd_plugin] = rgbd_plugin_process_frame(img, depth_img, temp_params.c, temp_params.bb, rgbd_plugin);
            
            if(rgbd_plugin.occlusion)
                if(rgbd_plugin.num_occluded_frames == 0)
                    
                    if(max(size(rgbd_plugin.consistent_centers)) == rgbd_plugin.successful_frame_limit) % Valid consistency measure
                        for s=2:rgbd_plugin.successful_frame_limit
                            rgbd_plugin.consistent_velocity(:,:) = rgbd_plugin.consistent_velocity(:,:) + abs(rgbd_plugin.consistent_centers(s,:) - rgbd_plugin.consistent_centers(s-1, :));
                        end
                        
                        rgbd_plugin.consistent_velocity = rgbd_plugin.consistent_velocity / rgbd_plugin.successful_frame_limit;
                    end
                end
                fprintf(fileID, '%s,%s,%s,%s\n', 'NaN', 'NaN', 'NaN', 'NaN');
                
                if(rgbd_plugin.visualize)
                    rgbd_plugin = visualize_all(rgbd_plugin, img, temp_params.bb);
                end
                
                continue;
            else
                fprintf(fileID, '%f,%f,%f,%f\n', tracker.bb(1), tracker.bb(2), tracker.bb(1) + tracker.bb(3), tracker.bb(2) + tracker.bb(4));
            end
            
            if(rgbd_plugin.visualize)
                rgbd_plugin = visualize_all(rgbd_plugin, img, temp_params.bb);
            end
            [tracker] = tracker_update(tracker, img, temp_params, rgbd_plugin.mask);
            
        else
            rgbd_plugin.num_occluded_frames = rgbd_plugin.num_occluded_frames + 1;
            [rgbd_plugin, c, bb, resp_score, temp_params] = occlusion_recovery(rgbd_plugin, tracker, img);
            
            tracker.c = c;
            tracker.bb = bb;
            
            tracker.currentScaleFactor = temp_params.currentScaleFactor;
            rgbd_plugin.currentScaleFactor = temp_params.currentScaleFactor;
            if(rgbd_plugin.occlusion == false)
                rgbd_plugin.response(rgbd_plugin.frame_counter) = resp_score;
                
                if(mod(rgbd_plugin.frame_counter, rgbd_plugin.history) == 0)
                    rgbd_plugin.frame_counter = 1;
                else
                    rgbd_plugin.frame_counter = rgbd_plugin.frame_counter + 1;
                end
                [rgbd_plugin] = rgbd_plugin_process_frame(img, depth_img, c, bb, rgbd_plugin);
                
                if(rgbd_plugin.occlusion == false)
                    rgbd_plugin.occlusion_recovery_frame_no = rgbd_plugin.frame_no;
                    [tracker] = tracker_update(tracker, img, temp_params, rgbd_plugin.mask);
                end
            end
            if(rgbd_plugin.visualize)
                rgbd_plugin = visualize_all(rgbd_plugin, img, bb);
            end
            if(rgbd_plugin.occlusion == false)
                fprintf(fileID, '%f,%f,%f,%f\n', bb(1), bb(2), bb(1) + bb(3), bb(2) + bb(4));
            else
                fprintf(fileID, '%s,%s,%s,%s\n', 'NaN', 'NaN', 'NaN', 'NaN');
            end
        end
    end
    fclose(fileID);
    close all
    
end