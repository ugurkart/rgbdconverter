function [rgbd_plugin] = visualize_all(rgbd_plugin, rgb_img, bb)

patch_center_x = size(rgbd_plugin.target_region, 2) / 2;
patch_center_y = size(rgbd_plugin.target_region, 1) / 2;
object_region_x = max(1, patch_center_x - (bb(3) / 2));
object_region_y = max(1, patch_center_y - (bb(4) / 2));

obj_reg = [object_region_x object_region_y bb(3) bb(4)];

if size(rgb_img,3) == 1
    rgb_img = repmat(rgb_img, [1 1 3]);
end
if (rgbd_plugin.frame_no == 1)  %first frame, create GUI
    rgbd_plugin.fig_handle = figure('Name', 'Tracking');
    set(rgbd_plugin.fig_handle, 'Position', [100, 100, 2 * size(rgb_img,2), 2 * size(rgb_img,1)]);
else
    figure(rgbd_plugin.fig_handle);
end
colormap jet;
subplot(2,3,1)
imagesc(rgbd_plugin.target_region);
hold on;
text(15, 25, int2str(rgbd_plugin.frame_no), 'color', [0 1 1], 'FontSize', 15, 'FontWeight', 'bold');
if(rgbd_plugin.occlusion)
    rectangle('Position',obj_reg, 'EdgeColor','r', 'LineWidth',2);
    text(10, 40, 'Occlusion', 'color', [1 0 0]);
else
    rectangle('Position',obj_reg, 'EdgeColor','g', 'LineWidth',2);
end
hold off;

if(rgbd_plugin.occlusion)
    subplot(2,3,2)
    imagesc(rgbd_plugin.search_region)
    axis off
    hold on
    text(15, 25, 'Search Region', 'Color','g', 'FontSize', 15, 'FontWeight', 'bold');
    hold off
else
    subplot(2,3,2)
    imagesc(rgbd_plugin.mask)
    axis off
    hold on
    text(15, 25, 'Mask', 'Color','g', 'FontSize', 15, 'FontWeight', 'bold');
    hold off
end
subplot(2,3,3)
imagesc(rgbd_plugin.depth_p)
hold on
text(15, 25, 'Depth', 'Color','g', 'FontSize', 15, 'FontWeight', 'bold');
axis off
hold off

subplot(2,3,4)
imagesc(rgbd_plugin.color_p)
hold on
text(15, 25, 'Color', 'Color','g', 'FontSize', 15, 'FontWeight', 'bold');
axis off
hold off

subplot(2,3,5)
plot(rgbd_plugin.foreground_depth_hist,'LineWidth',3)
hold on
legend('Foreground Depth Hist');
text(15, 25, 'Foreground Depth Hist', 'Color','g', 'FontSize', 15, 'FontWeight', 'bold');
axis on
hold off

subplot(2,3,6)
plot(rgbd_plugin.background_depth_hist,'LineWidth',3)
hold on
legend('Background Depth Hist');
text(15, 25, 'Background Depth Hist', 'Color','g', 'FontSize', 15, 'FontWeight', 'bold');
axis on
hold off

drawnow

end

