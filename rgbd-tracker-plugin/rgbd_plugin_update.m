function [rgbd_plugin] = rgbd_plugin_update(rgbd_plugin, c, bb, scaleFactor, resp_score)
rgbd_plugin.c = c;
rgbd_plugin.bb = bb;
rgbd_plugin.currentScaleFactor = scaleFactor;
end

