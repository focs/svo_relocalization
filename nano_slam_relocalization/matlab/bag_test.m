

clear rosbag_wrapper;
clear ros.Bag;
clear all

bagfile = '~/dense_input_data.bag';
bag = ros.Bag(bagfile);

%% 

msgs = bag.readAll('/nanoslam/dense_input');

%%

gaussian_filter_mask = fspecial('gaussian',3, 2.5);

im_subsampled_blured = {};
im_subsampled_blured_0mean = {};

for img_idx = 1:length(msgs)
    im = reshape(msgs{img_idx}.image.data, [msgs{img_idx}.image.width msgs{img_idx}.image.height]);
    
    im_subsampled = (imresize(im2double(im), [40,30]));
    im_subsampled_blured{img_idx} = imfilter(im_subsampled, gaussian_filter_mask);
    
    tmp = im_subsampled_blured{img_idx};
    im_subsampled_blured_0mean{img_idx} = im_subsampled_blured{img_idx} - mean(tmp(:));
%     disp(['frame id ' num2str(msgs{img_idx}.frame_id)]);
%     imshow(im)
%     
%     msgs{img_idx}.pose.position
%     msgs{img_idx}.pose.orientation
%     
%     pause(0.5)
end

%%

tic;
search_idx = 350;
square_difference = [];
for img_idx = 1:length(im_subsampled_blured_0mean)
    if (img_idx ~= search_idx)
        im_substraction = (im_subsampled_blured_0mean{img_idx} ...
                           - im_subsampled_blured_0mean{search_idx}).^2;
        square_difference(img_idx) = sum(im_substraction(:));
    end
end

plot(square_difference);
square_difference(search_idx) = Inf;

[min_value min_idx] = min(square_difference);
toc;
plot(square_difference);



%% 

clear r f

% Instantiate cost function class
r = RotationCostFunction;
r.im_ref = im_subsampled_blured{search_idx};
r.im_new = im_subsampled_blured{min_idx};

% r.getCost([0 0 0 0])

% Start minimization
x0 = [0 0 0 0];
f = @r.getCost;
options = optimset('LargeScale', 'off');
tic;
[x,fval,exitflag,output] = fminunc(f, x0, options);
toc;

figure; 
subplot(1,2,1); imshow(abs(r.im_ref - r.getTransformedImage(x)));
subplot(1,2,2); imshow(abs(r.im_ref - r.im_new));
