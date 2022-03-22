function [xx,tt,yy] = cobstacle(PG,finger)
deltath = 0.05;
tt = -pi():deltath:pi;
comTOvert = PG.vertex-PG.com;
comTOvertDist = comTOvert.'*comTOvert;
maxdist = sqrt(max(diag(comTOvertDist)));
xx = linspace(finger(1)-maxdist,finger(1)+maxdist,100);
yy = zeros(length(tt),length(xx));
for th = 1:length(tt)
    theta = wrapToPi(tt(th)+pi());
    R = [cos(theta) -sin(theta);sin(theta) cos(theta)];
    vert = finger+R*comTOvert;
    yy(th,:) = firstCollision(vert,xx).';
end
end

function yy = firstCollision(vert,xx)
yy=-100*ones(size(xx));
yytemp = yy;
current = find(vert(1,:)==max(vert(1,:)),1,'last');
yy(xx>max(vert(1,:)))=nan;
vert = [vert(:,current:end),vert(:,1:current-1)];
last = find(vert(1,:)==min(vert(1,:)),1,'first');
yy(xx<min(vert(1,:)))=nan;
for i = 1:last-1
    if vert(1,i+1)>vert(1,i) % upper limit must be met with edges going right-to-left
        continue
    end
    xcond = (xx<=vert(1,i))&(xx>=vert(1,i+1));
    xdist = vert(1,i)-vert(1,i+1);
    ydist = vert(2,i)-vert(2,i+1);
    m = ydist/xdist;
    yytemp(xcond) = (xx(xcond)-vert(1,i+1))*m+vert(2,i+1);
    ycond = yytemp>yy;
    yy = yy.*(~ycond)+yytemp.*ycond;
end
end