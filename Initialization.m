clc;
clear all;
  clf;
% clear all;
M=randperm(4,4); 
cars=length(M);%no of taxis
N=14; %The set of all jobs 
n=50;
p=randi(n,2,N);%pickup points
d=randi(n,2,N);%drop points 
[D,idxp1]=datasample(p(1,:),N);
scatter(p(1,:),p(2,:),'b','filled');%plotting the pick up points
hold on;
scatter(d(1,:),d(2,:),'o');%plotting the drop off points
hold on;
d0=[0;0];
cluster_pop  =  cluster_intialization(cars,N);


%% CLUSTERING

empty_cluster.cluster_child = [];%zeros(cars,length(p)/4);
cluster=repmat(empty_cluster,100,1);
empty_cluster.cluster_child1 = []; %zeros(cars*2,length(p)/2);
Fitness=zeros(100,1); 
for e=1:2
 for s = 1:100
     k=1;
      INDEX=randperm(numel(p)/2);
      EachCarDist=zeros(4,1);
%       taxy=zeros(4,4);
    for j=1:cars 
        cluster_pop(s).SUMS(j,:)=sum(cluster_pop(s).cluster_child3(j,:))-4; % 4 is passenger per car
         
            Q=find(cluster_pop(s).cluster_child3(j,:));
            if  cluster_pop(s).SUMS(j,:)>0 && j<cars
                   
                   for x=1:cluster_pop(s).SUMS(j,:)
                   cluster_pop(s).cluster_child3(j,Q(x))=0;
                   cluster_pop(s).cluster_child3(j+1,Q(x))=1;
                   cluster_pop(s).SUMS(j,:)=sum(cluster_pop(s).cluster_child3(j,:))-4;
                   cluster_pop(s).SUMS(j+1,:)=sum(cluster_pop(s).cluster_child3(j+1,:))-4;
                   end
            elseif  cluster_pop(s).SUMS(j,:)>0 && j==cars
                   for x=1:cluster_pop(s).SUMS(j,:)
                    cluster_pop(s).cluster_child3(j,Q(x))=0;
                    cluster_pop(s).cluster_child3(1,Q(x))=1;
                   cluster_pop(s).SUMS(j,:)=sum(cluster_pop(s).cluster_child3(j,:))-4;
                   cluster_pop(s).SUMS(1,:)=sum(cluster_pop(s).cluster_child3(1,:))-4;
                   end
            end        
         
        
       CLUSTER=find(cluster_pop(s).cluster_child3(j,:));
       if isempty(CLUSTER)
           j=j+1;
            CLUSTER=find(cluster_pop(s).cluster_child3(j,:));
                    
       end
%             for i=1:numel(p)/(cars*2) %4 for 4 number of taxis,this loop is not needed 
% %                 taxi(j,i)=p(1,INDEX(k)); % dividing the values of p according to INDEX into taxi
%                 taxy(j,i)=INDEX(1,k); %indices of the values from p into taxi
%                 k=k+1;
%             end
              F1=[];
                idx=[];
                i=1;
            for t=1:2:(length(CLUSTER)*2) 
    %             [c(i,:),idx(i,:)]=%datasample(p(1,:),4);
             %placing pick up points on the odd positions and drop off at even indices
                F1(:,t)=p(:,CLUSTER(:,i));%p(:,taxy(j,i));%p(:,idx(1,i));
                F1(:,t+1)=d(:,CLUSTER(:,i));%d(:,taxy(j,i));%d(:,idx(1,i));
                i=i+1;
            end
            
                cluster(s).cluster_child1(((2*j)-1):2*j,:)=[F1(1:2,:),zeros(2,(2*(N-length(CLUSTER))))];  %at odd are the pick ups,even are the drop offs
             
                F2=zeros(2,1);
                F=[F2 F1];%cluster(s).cluster_child1(((2*j)-1):2*j,:)
%                 F3(((2*j)-1):2*j,:)=F1; %pick and drop off routes for each car without depot
                
 cluster(s).cluster_child(j,:)=[CLUSTER,zeros(1,N-length(CLUSTER))]; %index values 
 cluster(s).cluster_child3=cluster_pop(s).cluster_child3;
 my_ga = struct();
my_ga.xy          = F';
my_ga.dmat        = [];
my_ga.popSize     = 64;
my_ga.numIter     = 10;
% my_ga.showProg    = true;
% my_ga.showResult  = true;
% my_ga.showWaitbar = false;
final_output = tsp_ga(my_ga);
cluster(s).EachCarDist(j,:)=final_output.minDist;
cluster(s).Fitness=1/sum(cluster(s).EachCarDist(:,1));
Fitness(s,:)=1/sum(cluster(s).EachCarDist(:,1));
cluster(s).rte(j,:)=[final_output.rte,ones(1,(2*(N-length(CLUSTER))))];



    end  
  % final_output = struct();
% disp(final_output.minDist);
 end 
end
%% Crossover Clustering
[Parent_Dist,I]=maxk(Fitness,40);
new_cluster=struct();
update_cluster=struct();

for j=1:40
 update_cluster = cluster(j).cluster_child3;
 new_cluster(j).cluster_child=update_cluster;
 [size_cluster,~]=size(update_cluster);
 parent_combination = randperm(size_cluster,size_cluster);
 
for i=1:2:size_cluster
    parent1=  update_cluster(parent_combination(i),:);%cluster(I(j,:)).cluster_child(parent_combination(i),:);
    parent2= update_cluster(parent_combination(i+1),:);%cluster(I(j,:)).cluster_child(parent_combination(i+1),:);
    split = randperm(length(parent1),1);
    child1= [parent1(:,1:split-1) parent2(:,split:end)];
    child2 = [parent2(:,1:split-1) parent1(:,split:end)];
   
     if sum(child1)==0 || sum(child2)==0
         tmpChild=child1;
         child1(:,1:(length(child1)/2))=child2(:,(length(child2)/2)+1:end);
         child2(:,(length(child2)/2)+1:end)=tmpChild(:,1:(length(child1)/2));
     end
    new_cluster(j).cluster_child1(i,:)= child1;
    new_cluster(j).cluster_child1(i+1,:)= child2;
    
     new_cluster(j).SUMS(i,:)=sum(new_cluster(j).cluster_child1(i,:))-4; % 4 is passenger per car
     new_cluster(j).SUMS(i+1,:)=sum(new_cluster(j).cluster_child1(i+1,:))-4; % 4 is passenger per car
end 
for e=1:2
        for i=1:cars
            c=find(new_cluster(j).cluster_child1(i,:));
            if new_cluster(j).SUMS(i,:)>0 && i<cars
                   
                   for x=1:new_cluster(j).SUMS(i,:)
                   new_cluster(j).cluster_child1(i,c(x))=0;
                   new_cluster(j).cluster_child1(i+1,c(x))=1;
                   new_cluster(j).SUMS(i,:)=sum(new_cluster(j).cluster_child1(i,:))-4;
                   new_cluster(j).SUMS(i+1,:)=sum(new_cluster(j).cluster_child1(i+1,:))-4;
                   end
           elseif new_cluster(j).SUMS(i,:)>0 && i==cars
                   for x=1:new_cluster(j).SUMS(i,:)
                   new_cluster(j).cluster_child1(i,c(x))=0;
                   new_cluster(j).cluster_child1(1,c(x))=1;
                   new_cluster(j).SUMS(i,:)=sum(new_cluster(j).cluster_child1(i,:))-4;
                   new_cluster(j).SUMS(1,:)=sum(new_cluster(j).cluster_child1(1,:))-4;
                   end
            end
             
            CLUSTER1=find(new_cluster(j).cluster_child1(i,:));
            if isempty(CLUSTER1) 
                  continue
            end
                    F1=zeros(2,N/2);
                    idx=[];
                    k=1;
                   POINTS=[]; 
                for t=1:2:(length(CLUSTER1)*2) 
                   %placing pick up points on the odd positions and drop off at even indices
                   POINTS(:,t)=p(:,CLUSTER1(:,k));  %p(:,idx(1,i));
                   POINTS(:,t+1)=d(:,CLUSTER1(:,k));  %d(:,idx(1,i));
                   k=k+1;
                end

             
          new_cluster(j).POINTS(((2*i)-1):2*i,:)=[POINTS(1:2,:),zeros(2,(2*(N-length(CLUSTER1))))]; 
F2=zeros(2,1);
F=[F2 POINTS(1:2,:)];
my_ga = struct();
my_ga.xy          = F';
my_ga.dmat        = [];
my_ga.popSize     = 64;
my_ga.numIter     = 10;
% my_ga.showProg    = true;
% my_ga.showResult  = true;
% my_ga.showWaitbar = false;
final_output = tsp_ga(my_ga);
new_cluster(j).EachCarDist(i,:)=final_output.minDist;
new_cluster(j).Fitness=1/sum(new_cluster(j).EachCarDist(:,1));
Fitness1(j,:)=1/sum(new_cluster(j).EachCarDist(:,1));
new_cluster(j).rte(i,:)=[final_output.rte,ones(1,(2*(N-length(CLUSTER1))))];
        end  
end
end

[Child_Dist,I1]=maxk(Fitness1,40);
 if Child_Dist(1)<Parent_Dist(1)
        BestCluster=cluster(I).rte;
        points=zeros(8,1);
        points=[points cluster(I(1)).cluster_child1];
      
        hAx = gca;
        plot(hAx,points(1,BestCluster(1,:)),points(2,BestCluster(1,:)),'r.-');hold on;
        plot(hAx,points(3,BestCluster(1,:)),points(4,BestCluster(1,:)),'g.-');hold on;
        plot(hAx,points(5,BestCluster(1,:)),points(6,BestCluster(1,:)),'b.-');hold on;
        plot(hAx,points(7,BestCluster(1,:)),points(8,BestCluster(1,:)),'m.-');
        xlabel("x-coordinate");
          ylabel("y-coordinate");
        title('Routes');
        legend('Pick up point','Drop off point','Car 1', 'Car 2', 'Car 3', 'Car 4')
         drawnow;
 else
       BestCluster=new_cluster(I1).rte;
       points=zeros(8,1);
       points=[points new_cluster(I1(1)).POINTS];
      
       hAx = gca;
       plot(hAx,points(1,BestCluster(1,:)),points(2,BestCluster(1,:)),'r.-');hold on;
       plot(hAx,points(3,BestCluster(1,:)),points(4,BestCluster(1,:)),'g.-');hold on;
       plot(hAx,points(5,BestCluster(1,:)),points(6,BestCluster(1,:)),'b.-');hold on;
       plot(hAx,points(7,BestCluster(1,:)),points(8,BestCluster(1,:)),'m.-');
       xlabel("x-coordinate");
        ylabel("y-coordinate");
        title('Routes');
        legend('Pick up point','Drop off point','Car 1', 'Car 2', 'Car 3', 'Car 4')
        drawnow;
 end


%%cluster initialization

function cluster  = cluster_intialization(cars,N)
empty_cluster.cluster_child = zeros(cars,N);
cluster=repmat(empty_cluster,100,1);

    for i = 1:100
    cluster(i).cluster_child4=zeros(cars,N);
    cluster(i).cluster_child3=zeros(cars,N);
    initial_assign = randperm(N);
    split_index = sort(randperm(N,cars-1));
    split_index_update = [1,split_index,N+1];

        for j = 1:cars
           cluster(i).cluster_child3(j,initial_assign(split_index_update(j):split_index_update(j+1)-1))=1;
           cluster(i).cluster_child4(j,:)=[initial_assign(split_index_update(j):split_index_update(j+1)-1),zeros(1,N-length(initial_assign(split_index_update(j):split_index_update(j+1)-1)))];
           
        end
        
      end
end

