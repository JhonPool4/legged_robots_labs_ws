function [rows, cols, list_index] = get_subplot_index(cnt_img_h, cnt_img_v, img_index, img_size, space_size, legend_flag)
  
if legend_flag
    legend_space =1;
else
    legend_space =0;
end

list_index = [];
h_space_size = 2*space_size;
v_space_size = 2*space_size;

rows = legend_space + cnt_img_v*(img_size+v_space_size);
cols  = img_size*(cnt_img_h) +h_space_size*(cnt_img_h+1);

v_img_index = ceil( img_index/cnt_img_h);
h_img_index = img_index - cnt_img_h*(v_img_index-1);

v_plot_depth = legend_space + (v_img_index-1)*(img_size+v_space_size);
h_plot_depth = h_space_size +(h_img_index-1)*(img_size+h_space_size);

for img_depth=(v_plot_depth):(v_plot_depth+img_size-1)
    % p0 = image_line + image_index
    p0 = 1+ (img_depth)*cols+ h_plot_depth;
    for j = p0:(p0+img_size-1)
        list_index(end+1) = j;
    end
end
%list_index
end