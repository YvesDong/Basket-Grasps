function drawPolygon2fGrasp(PG,s1,s2,theta)
% s1 = 1.184;
% s2 = [];
% theta = -1.809;% -pi;
% f2 = [0;0];
% f1 = [-0.2;-0.03];
figure
f1 = [0;3];
f1rel = PG.get('1Pos',s1);
f2rel = PG.get('1Pos',s2);
axle = f1rel;
d = f1-f1rel;
R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
f2 = f1 + R*(f2rel-f1rel);

PG.drawPolygonMoved(axle,theta,d,'k');
PG.drawPolygonNode(theta,s1,s2,f1,f2);
hold on
fingerline = [f1,f2];
middlepoint = mean(fingerline,2);
direct = (fingerline-repmat(middlepoint,[1,2]))/0.9;
quiver(middlepoint(1)*[1,1],middlepoint(2)*[1,1],direct(1,:),direct(2,:),'k')
text(middlepoint(1),middlepoint(2)+0.2,'$\sigma$','Interpreter','latex','fontsize',18)
plot([middlepoint(1) middlepoint(1)+1],[middlepoint(2) middlepoint(2)],'k--')
text(middlepoint(1)+0.2,middlepoint(2),'$\phi$','Interpreter','latex','fontsize',18)
end