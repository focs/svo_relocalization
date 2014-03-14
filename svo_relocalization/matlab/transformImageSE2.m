
function im_transformed = transformImageSE2 (im, p)

alpha = p(1);
t1 = p(2);
t2 = p(3);
t = [1 0 -size(im,2)/2; 0 1 -size(im,1)/2; 0 0 1];

full_trans = pinv(t)*[cos(alpha) sin(alpha) t1; 
                     -sin(alpha) cos(alpha) t2;
                     0 0 1]*t;
full_trans (3,1:3) = [0 0 1];     


% Create SE(2) transform
tform = maketform('affine', full_trans'); 

% Apply transform on the image
%im_new_trans = imtransform(obj.im_new,tform);
im_transformed = imtransform(im, tform, ...
    'XData',[1 size(im,2)],...
    'YData',[1 size(im,1)]);
            
end