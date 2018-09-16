function [ possible_occlusion ] = analyze_mean_depth( tracker, current_mean_depth )
% Compare the latest mean depth extracted from the object region to 
% figure out possible occlusions

if(nnz(tracker.response) < tracker.num_depth_hist_history)
    max_index = tracker.frame_counter - 1;
else
    max_index = tracker.num_depth_hist_history;
end

cumulative_mean = mean(mean(tracker.mean_depth(1:max_index)));

diff_mean = current_mean_depth - cumulative_mean;

diff_percentage = double(diff_mean) / double(cumulative_mean);

% depth_percentage = diff_percentage

possible_occlusion = false;
if((abs(diff_percentage) > tracker.peak_percentage_depth) && (diff_mean < 0.0))
    possible_occlusion = true;
end

end

