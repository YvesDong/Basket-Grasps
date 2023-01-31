clear all
close all
res = 30; % resolution
% starting_node = [2,12]; % TODO: ??? (all cases work with diff 2nd no.)
%% Plot a new polygon
% load test.mat
% object = [object(:,7:8) object(:,1:6)];
% object = [object(:,1:3) [-0.15;0.23] [-0.1;0.21] object(:,4:8)];
% PG = Polygon(object,0*com);
% polyDrawing = PG.drawPolygon();

i=1;
texthandle = [];
flag = true;
figure
grid on
hold on
% imshow([pwd '\Examples for research proposal\Gun\snip.png'])
hold on
set(gca,'visible',1)
% xlim = get(gca,'xlim');
% ylim = get(gca,'ylim');
axis([-5 5 -5 5])
xv = [min(xlim),min(xlim)+diff(xlim)/10,min(xlim)+diff(xlim)/10,min(xlim)];
yv = [min(ylim),min(ylim),min(ylim)+diff(ylim)/10,min(ylim)+diff(ylim)/10];
pos = [xv(1),yv(1),max(xv)-min(xv),max(yv)-min(yv)];
rectangle('Position',pos,'faceColor',[1 1 0 0.2])
texthandle = text(mean(xv),mean(yv),'complete')

while flag
%     if ~isempty(texthandle)
%         delete(texthandle);
%     end
%     texthandle = text(0,5,num2str(i));
    [object_x(i),object_y(i)] = ginput(1);
    
    if inpolygon(object_x(i),object_y(i),xv,yv)
        flag = false;
        object_x(end) = [];
        object_y(end) = [];
        break;
    end
    plot(object_x(i),object_y(i),'.k');
    if i>1
      plot(object_x(i-1:i),object_y(i-1:i),'k'); 
    end

    i = i+1;
end
plot([object_x(1),object_x(end)],[object_y(1),object_y(end)],'k'); 
delete(texthandle);
texthandle = text(0,5,'center of mass');
[com(1),com(2)] = ginput(1);
% plot(com(1),com(2),'*r');

object = [object_x;object_y];
com = com.';

% gun
% object = 0.8*[1,10;26,10;21,1;29,9;32,0;32,12;36,12;40,8;41,9;35,14;31,16;1,16].';%32,14;31,14
% com = 0.8*[27;10];

%%
% Round_object = round(object,2);
% % com(2) = max(Round_object(2,:))-com(2);
% % Round_object(2,:) = (max(Round_object(2,:))-Round_object(2,:));
% figure
% hold on
% plot(Round_object(1,:),Round_object(2,:))
% plot(com(1),com(2),'+')
% axis equal

save('test.mat','object','com')

% %% Find equilibrium grasps
% tic
PG = Polygon(object,com);
subFolderName = 'examples/test'; % where to save results
[PG,~,X] = PG.findBdyVariable(res);

[s1,s2,~] = PG.Eqcheck();
EqCurves = PG.EqCurveFinder();
for i=1:numel(EqCurves)
    curve = EqCurves{i};
    s1 = [s1, linspace(curve(1,1),curve(1,2),res)];
    s2 = [s2, linspace(curve(2,1),curve(2,2),res)];
    s1 = [s1, linspace(curve(2,1),curve(2,2),res)];
    s2 = [s2, linspace(curve(1,1),curve(1,2),res)];
end

% Check grasps for minimum/maximum (basket grasp/non BG)
[BG,NBG] = PG.StatusSeparate(s1,s2,X); % basket grasp / non basket grasp
timeSynesthise = toc;


%% Analysis of one BG in the BG sets
n = round(size(BG,2));
theta = 0;
ss1 = BG(1, n); % (ss1, ss2) is one BG
ss2 = BG(2, n);
f1rel = PG.get('1Pos',ss1);
f2rel = PG.get('1Pos',ss2);
R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
f1 = [0;0];
f2 = f1 + R*(f2rel-f1rel);
% f1 = [11.4;13.9];
% f2 = [18.2;16.09];
basepos = [f1,f2];

%%
% Find equilibrium grasps
% [PG,S,X,THL,CVL,~] = PG.findBdyVariable3(res);
% [PG,S,X] = PG.findBdyVariable(res);

% identify nodes, create double-support contours
Sigma=inter_finger_distance(X,X);
sig = norm(f2-f1);
[cont_original] = PG.GetSigmaContours(Sigma,sig);
cont = PG.CleanContour(cont_original,basepos);

[ds_max,ds_min,ds_virtual] = PG.DSNodes(cont);
[ss_max,ss_min,ss_saddle] = PG.SSNodes();
nodes{1} = ds_max;
nodes{2} = ds_min;
nodes{3} = ds_virtual;
nodes{4} = ss_max;
nodes{5} = ss_min;
nodes{6} = ss_saddle;

BGS = [ss1;ss2];
diffToBG = ds_min(:,1:2)-BGS.';
diffToBG2 = diffToBG.^2;
distToBG = diffToBG2(:,1)+diffToBG2(:,2);
starting_node = [2,find(distToBG == min(distToBG),1,'first')];

nodes = check_SS_for_DS(PG,nodes,f1,100*(f2-f1)); %function checks SS_nodes for penetration (of the other finger),
% each node appears once for each finger it is relevant for, with the relevant finger index at the end 
% save([pwd '/' subFolderName '/PreGraph.mat'],'PG','nodes','cont','starting_node','f1','f2');
%% draw the contact space
loadfromfile = 0;
if loadfromfile == 1
    for finger = 1:2
        if exist([pwd '/' subFolderName '/Nodes s' num2str(finger) '.fig'],'file')
            graphfig{finger} = openfig([pwd '/' subFolderName '/Nodes s' num2str(finger) '.fig']); 
%             graphfig{finger} = openfig([pwd '/' subFolderName '/Polygon Escape Graph' num2str(finger) '.fig']); 
        end
    end
end
if ~exist('graphfig','var')
    graphfig = cell(1,2);
end

for finger = 1:2
    if isempty(graphfig{finger})||~isgraphics(graphfig{finger})
            otherFingerRelative = basepos(:,3-finger)-basepos(:,finger);
            contactHeight = basepos(2,finger);
            graphfig{finger} = plotGraphNodes(PG,cont,finger,nodes,otherFingerRelative,contactHeight);

    end
    set(graphfig{finger},'Position',[(finger-1)*960,41,960,963])
    set(gcf,'renderer','Painters')
end
%%
% tic
% [OpenList,ClosedList,A,index_list,type_list,path_list,FingerMatrix,pathwayMatrix] = GraphConstruction(PG,cont,nodes,f1,f2,graphfig);
% toc
% save([pwd '/' subFolderName '/completeGraph.mat'],'ClosedList','A','index_list','type_list','path_list','FingerMatrix','pathwayMatrix');
%% Graphical presentation of the results:
% load([pwd '/' subFolderName '/completeGraph.mat'])
% graphMarkers = plotResults_fullgraph(nodes,cont,ClosedList,index_list,type_list,A,FingerMatrix,pathwayMatrix,graphfig,subFolderName);
%%
for finger = 1:2

%             graphfig{finger} = openfig([pwd '/' subFolderName '/Fullgraph' num2str(finger) '.fig']);

%            graphfig{finger} = openfig([pwd '/' subFolderName '/Nodes s' num2str(finger) '.fig']);

%     set(graphfig{finger},'Position',[(finger-1)*960,41,960,963])
end

%% Algorithm Run:

[Open_list,Closed_list,A,index_list,type_list,path_list,FingerMatrix,pathwayMatrix] = GraphSearch(PG,cont,nodes,starting_node,f1,f2,graphfig);



%%
plotResults_graphSearch(nodes,cont,Closed_list,index_list,type_list,path_list,FingerMatrix,pathwayMatrix,graphfig,subFolderName)

%%
[U,delU,pp] = plotResults_pathPositionsNew(PG,nodes,Closed_list,index_list,type_list,path_list,basepos,subFolderName)
%%
save([pwd '/' subFolderName '/AlgResult.mat'],'U','delU','pp','PG','Open_list','Closed_list','A','index_list','type_list','path_list','FingerMatrix','pathwayMatrix','nodes','cont','basepos','starting_node')
% plotResults_NotpathPositions(PG,nodes,Closed_list,index_list,type_list,path_list,basepos,subFolderName)

%%
for drawResults=1:0 %for loop to allow folding
theta_index = [4,4,4,3,3,3];
if ~exist('graphMarkers','var')
graphMarkers = cell(0);
end
removegraphics = [];
for i=1:numel(graphMarkers)
    if ~isgraphics(graphMarkers{i})
        removegraphics = [removegraphics,i]; %#ok<*AGROW>
    end
end
graphMarkers(removegraphics) = [];
hold on
for i=1:length(Closed_list) % mark the current node of each step of the algorithm
    finger = [];
    cur_node_ind=Closed_list(i);
    connect_ind = find(path_list==cur_node_ind);
    cur_node_ind_intype = index_list(cur_node_ind);
    cur_node_type = type_list(Closed_list(i));
    node_par = nodes{cur_node_type}(cur_node_ind_intype,:);
    s_origin =node_par(1);
    theta_origin = node_par(theta_index(cur_node_type));
    if cur_node_type<4
        set(groot,'CurrentFigure',graphfig{1})
        hold on
        graphMarkers{end+1} = plot(s_origin,theta_origin,'ok','markerSize',14,'lineWidth',3); %#ok<*SAGROW>
        s_origin(2) =node_par(2);
        set(groot,'CurrentFigure',graphfig{2})
        hold on
        graphMarkers{end+1} = plot(s_origin(2),theta_origin,'ok','markerSize',14,'lineWidth',3);
    else
        finger = node_par(end);
        set(groot,'CurrentFigure',graphfig{finger})
        hold on
        graphMarkers{end+1} = plot(s_origin,theta_origin,'ok','markerSize',14,'lineWidth',3);
    end
    for j = 1:length(connect_ind) % mark each neighbor of the current node and connect them
        node_ind = connect_ind(j);
        pathway = pathwayMatrix{cur_node_ind,node_ind};
        node_ind_intype = index_list(node_ind);
        node_type = type_list(node_ind);
        node_par = nodes{node_type}(node_ind_intype,:);
        s =node_par(1);
        theta = node_par(theta_index(node_type));
        if node_type<4 % next node is DS_node
            s(2) =node_par(2);
            if isempty(finger) % current node is DS_node
%                 sMin1 = PG.S(PG.VL(find(PG.S(PG.VL)<s_origin(1),1,'last')));
%                 sMax1 = PG.S(PG.VL(find(PG.S(PG.VL)>s_origin(1),1,'first')));
%                 sMin2 = PG.S(PG.VL(find(PG.S(PG.VL)<s_origin(2),1,'last')));
%                 sMax2 = PG.S(PG.VL(find(PG.S(PG.VL)>s_origin(2),1,'first')));
                if any([1,3]==FingerMatrix(Closed_list(i),connect_ind(j)))
                    set(groot,'CurrentFigure',graphfig{1})
                    hold on
                    graphMarkers{end+1} = plot(s(1),theta,'or','markerSize',14);
                    graphMarkers = plotEdge(pathway,cont,graphMarkers,1);
%                     graphMarkers{end+1} = plot([s(1),s_origin(1)],[theta,theta_origin],'r-','lineWidth',4);
                    
                end
                if any([2,3]==FingerMatrix(Closed_list(i),connect_ind(j)))
                    set(groot,'CurrentFigure',graphfig{2})
                    hold on
                    graphMarkers{end+1} = plot(s(2),theta,'or','markerSize',14);
                    graphMarkers = plotEdge(pathway,cont,graphMarkers,2);
%                     graphMarkers{end+1} = plot([s(2),s_origin(2)],[theta,theta_origin],'r-','lineWidth',4);
                    
                end
            else % current node is SS_node
                set(groot,'CurrentFigure',graphfig{finger})
                hold on
                graphMarkers{end+1} = plot(s(finger),theta,'or','markerSize',14);
                graphMarkers = plotEdge(pathway,cont,graphMarkers,finger);
%                 graphMarkers{end+1} = plot([s(finger),s_origin],[theta,theta_origin],'r-','lineWidth',4);
                
            end
        else % next node is SS_node
            next_finger = node_par(end);
            set(groot,'CurrentFigure',graphfig{next_finger})
            hold on
            s_origin_temp = s_origin;
            if length(s_origin)>1
                s_origin_temp = s_origin(next_finger);
            end
            graphMarkers{end+1} = plot(s,theta,'or','markerSize',14);
            graphMarkers = plotEdge(pathway,cont,graphMarkers,next_finger);
%             graphMarkers{end+1} = plot([s,s_origin_temp],[theta,theta_origin],'r-','lineWidth',4);
            
        end
    end
end
hold off 
for finger = 1:2
saveas(graphfig{finger},[pwd '/' subFolderName '/Polygon Escape Graph' num2str(finger) '.bmp'])
saveas(graphfig{finger},[pwd '/' subFolderName '/Polygon Escape Graph' num2str(finger) '.fig'])
end
%% plot each position of the polygon along the path in separate figures
if 1
xpos = [(0:384:1536),(0:384:1536)];
ypos = [542*ones(1,5),41*ones(1,5)];
for i=1:length(Closed_list)
    f1t = f1;
    f2t = f2;
    connect_ind = find(path_list==Closed_list(i));
    node_ind = index_list(Closed_list(i));
    node_type = type_list(Closed_list(i));
    node_par = nodes{node_type}(node_ind,:);
    s_origin =node_par(1);
    theta_origin = node_par(theta_index(node_type));
    h_origin = node_par(theta_index(node_type)-1);
    if node_type>3 && node_par(end)==2
        f1t = f2;
        f2t = f1;
    end
    fig = drawPolygonNode(PG,s_origin,theta_origin,f1t,f2t);
    if i<=10
    set(fig,'Position',[xpos(i),ypos(i),384,461])
    end
    xlim([-6 6]);
    ylim([-10 6]);
    if i<length(Closed_list)
        title(['Node ' num2str(i) ', height = ,' num2str(h_origin)]);
    else
        title(['Node ' num2str(i) ', height = ,' num2str(h_origin) ', Final']);
    end
    saveas(fig,[pwd '/' subFolderName '/Node ' num2str(i) '.bmp']);
    saveas(fig,[pwd '/' subFolderName '/Node ' num2str(i) '.fig']);
end
end
%% Removing markings from the graphs to reuse them
if ~isempty(graphMarkers) && false
    for i = 1:numel(graphMarkers)
        delete(graphMarkers{i});
    end
end
end
%%
function graphMarkers = plotEdge(pathway,cont,graphMarkers,finger)
for i=1:numel(pathway)-1
    node1 = pathway{i}; %nodes here refer to points along the pathway, not necessarily nodes of the graph
    node2 = pathway{i+1};
    s1 = node1(1,1); theta1 = node1(1,3);
    s2 = node2(1,1); theta2 = node2(1,3);
    if length(node1)==3
        if abs(theta2-theta1)<pi() % if the pathway goes directly over the plane
            graphMarkers{end+1} = plot([s1,s2],[theta1,theta2],'r--','lineWidth',4);
        else %otherwise, it goes to +-pi, and comes from the other (over the cycle of theta)
            thetas = sort([theta1,theta2]);
            graphMarkers{end+1} = plot([s1,s2],[thetas(1),-pi()],'r--','lineWidth',4);
            graphMarkers{end+1} = plot([s1,s2],[thetas(2),pi()],'r--','lineWidth',4);
        end
        % need to add check for theta going the wrong direction
        % (delta_theta>pi)
    else
        seg = cont{node1(end-3)};
        if node1(end-2)>node2(end-2)
            graphMarkers{end+1} = plot(seg(finger,node2(end-2):8:node1(end-2)),seg(4,node2(end-2):8:node1(end-2)),'r--','lineWidth',6,'markerSize',6);
        else
            graphMarkers{end+1} = plot(seg(finger,node1(end-2):8:node2(end-2)),seg(4,node1(end-2):8:node2(end-2)),'r--','lineWidth',6,'markerSize',6);
        end
    end
    
end
end