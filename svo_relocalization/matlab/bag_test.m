

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

initial_diff = abs(r.im_ref - r.im_new);
optimized_diff = abs(r.im_ref - r.getTransformedImage(x));

disp(['sum of initial differences ' num2str(sum(initial_diff(:)))]);
disp(['sum of optimized differences ' num2str(sum(optimized_diff(:)))]);

figure; 
subplot(1,2,1); imshow(initial_diff);
subplot(1,2,2); imshow(optimized_diff);

%% Lucas kanard

% Template image
im_template = im2double(rgb2gray(imread('/opt/matlab2012a/toolbox/images/imdemos/onion.png')));
im_template = im2double((imread('/opt/matlab2012a/toolbox/images/imdemos/cameraman.tif')));

% Image
alpha = 0.01;
t1 = 5;
t2 = -2;
tform = maketform('affine', ...
                    [cos(alpha) sin(alpha) t1; 
                     -sin(alpha) cos(alpha) t2;
                     0 0 1]'); 
        
% Apply transform on the image
im = imtransform(im_template, tform, ...
                'XData',[1 size(im_template,2)],...
                'YData',[1 size(im_template,1)]);
            
% [p,I_roi,T_error]=LucasKanadeAffine(im,[0 0 0 0 0 0],im_template);

mask = true(size(im_template));
% mask(50:200, 50:200) = true;

profile on
tic
[im_final p] = myLucasKanade(im_template, im, mask);
toc
profile off

% Show results
subplot(2,1,1); imshow([im_template im im_final]);
subplot(2,1,2); imshow(mat2gray(im_final - im_template));

disp(['error: ' num2str(sum((im_final(:) - im_template(:)).^2))]);

 