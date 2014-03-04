

classdef RotationCostFunction
    properties
       im_ref
       im_new
    end
    
    methods
        function cost = getCost(obj, x)
            
            
%             im_ref_pad = padarray(obj.im_ref, ceil((size(im_new_trans) - size(obj.im_ref))/2), 'pre');
%             im_ref_pad = padarray(im_ref_pad, floor((size(im_new_trans) - size(obj.im_ref))/2), 'post');
%             im_ref_pad = obj.im_ref;
%             
%             mask = im_ref_pad & im_new_trans;
            
%             figure; 
%             subplot(1,2,1); imshow(im_new_trans);
%             subplot(1,2,2); imshow(obj.im_ref);

            im_new_light_trans = obj.getTransformedImage(x);
            
            cost = (im_new_light_trans - obj.im_ref).^2;
            cost = sum(cost(:));
            
        end
        
        function im_new_light_trans = getTransformedImage (obj, x)
            
            alpha = x(1); % SE(2) Rotation
            t1 = x(2); % SE(2) first dimension translation
            t2 = x(3); % SE(2) second dimension translation
            intencity_offset = x(4);
            
            % Apply light correction
            im_new_light = obj.im_new + intencity_offset;
            
            % Create SE(2) transform
            tform = maketform('affine', ...
                    [cos(alpha) sin(alpha) 0; 
                     -sin(alpha) cos(alpha) 0;
                     t1 t2 1]); 
        
            % Apply transform on the image
            %im_new_trans = imtransform(obj.im_new,tform);
            im_new_light_trans = imtransform(im_new_light, tform, ...
                'XData',[1 size(obj.im_new,2)],...
                'YData',[1 size(obj.im_new,1)]);
            
        end
    end
    
end