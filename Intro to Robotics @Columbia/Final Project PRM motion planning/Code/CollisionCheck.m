function out = CollisionCheck (fv1, fv2)
% Determine if two sets of triangular faces overlap

n1 = size (fv1, 2);
n2 = size (fv2, 2);
flag=zeros(n1,n2);
for i = 1:n1
    for j = 1:n2 
        m1=size(fv1(1,i).faces,1);
        for k1=1:m1
          if (separate_plane(fv1(1,i).vertices,fv2(1,j).vertices,fv1(1,i).faces(k1,:)))
              flag(i,j)=1;
          end
        end
        m2=size(fv2(1,j).faces,1);
        for k2=1:m2
          if (separate_plane(fv1(1,i).vertices,fv2(1,j).vertices,fv2(1,j).faces(k2,:)))
             flag(i,j)=1;
          end
        end
    end
end
out = true;
if(sum(sum(flag))==n1*n2)
    out=false;
end
