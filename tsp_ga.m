
function varargout = tsp_ga(varargin)
    
    % Initialize default configuration
%     p=randi(n,2,N);
    defaultConfig.xy          = [0,45,44;0,11,33]';%10*rand(5,2);%randi(100,2,16)';,15,45,37,34,6,37,21,11 ,47,9,41,46,40,41,30,45
    defaultConfig.dmat        = [];
    defaultConfig.popSize     = 4;%5
    defaultConfig.numIter     = 1e4;
    defaultConfig.showProg    = true;
    defaultConfig.showResult  = true;
    defaultConfig.showWaitbar = false;
    
    % Interpret user configuration inputs
    if ~nargin
        userConfig = struct();
    elseif isstruct(varargin{1})
        userConfig = varargin{1};
    else
        try
            userConfig = struct(varargin{:});
        catch
            error('Expected inputs are either a structure or parameter/value pairs');
        end
    end
    
    % Override default configuration with user inputs
    configStruct = get_config(defaultConfig,userConfig);
    
    % Extract configuration
    xy          = configStruct.xy;
    dmat        = configStruct.dmat;
    popSize     = configStruct.popSize;
    numIter     = configStruct.numIter;
    showProg    = configStruct.showProg;
    showResult  = configStruct.showResult;
    showWaitbar = configStruct.showWaitbar;
    if isempty(dmat)
        nPoints = size(xy,1);
        a = meshgrid(1:nPoints);
        dmat = reshape(sqrt(sum((xy(a,:)-xy(a',:)).^2,2)),nPoints,nPoints);
    end
    
    % Verify Inputs
    [N,~] = size(xy);
    [nr,nc] = size(dmat);
    if N ~= nr || N ~= nc
        error('Invalid XY or DMAT inputs!')
    end
    
%             XY=xy';
%         for i=2:2:8
%             if i==2
%                   f(1,1)={XY(1:2,i:i+1)};
%                   j=1;
%             else 
%                 j=j+1;
%                 f(1,j)={XY(1:2,i:i+1)};
%             end
%         end
        
      n=N;     
%     [~,m] =size(f);
    m=(length(xy)-1)/2;
    % Sanity Checks
    popSize     = 4*ceil(popSize/4);
    numIter     = max(1,round(real(numIter(1))));
    showProg    = logical(showProg(1));
    showResult  = logical(showResult(1));
    showWaitbar = logical(showWaitbar(1));
    
    % Initialize the Population
    pop1 = zeros(popSize,m);
    pop1(1,:) = (1:m);
    pop2=zeros(popSize,m);
    for k = 2:popSize
        pop1(k,:) = randperm(m);
    end
         t=1;
        for i=1:popSize
            k=1;
           
                for j=1:m
                   % pop(j:j+1,k:k+1)=f{1,pop1(j,i)}; %{XY(1:2,i:i+1)};
                pop2(t,k:k+1)=[2*pop1(i,j)-1,2*(pop1(i,j))];   
                k=k+2;
                 end
        t=t+1;
        end
    pop=pop2+1;
    
    % Run the GA
    globalMin = Inf;
    totalDist = zeros(1,popSize);%popSize
    distHistory = zeros(1,numIter);
    tmpPop = zeros(4,n-1);
    newPop = zeros(popSize,n-1);
%     if showProg
%         figure('Name','TSP_GA | Current Best Solution','Numbertitle','off');
%         hAx = gca;
%     end
%     if showWaitbar
%         hWait = waitbar(0,'Searching for near-optimal solution ...');
%     end
    for iter = 1:numIter
        % Evaluate Each Population Member (Calculate Total Distance)
        for p = 1:popSize%popSize % CHANGE the POP to INDEXES 
            d = dmat(1,pop(p,1)); % Closed Path
             for j = 2:m
                d = d + dmat(pop(p,j-1),pop(p,j));
            end
            d=d+dmat(pop(p,N-1),1);
            totalDist(p) = d;
        end
        
        % Find the Best Route in the Population
           [minDist,index] = min(totalDist);
%            [Distances,index1]=sort(totalDist);
        distHistory(iter) = minDist;
     if minDist < globalMin
            globalMin = minDist;
            optRoute = pop(index,:);
            
%             if showProg
% %                Plot the Best Route
                rte = [1,optRoute(1:N-1),1];
% %                 if dims > 2, plot3(hAx,xy(rte,1),xy(rte,2),xy(rte,3),'r.-');
% %                 else
%                     plot(hAx,xy(rte,1),xy(rte,2),'r.-');
%                      
%                 title(hAx,sprintf('Total Distance = %1.4f, Iteration = %d',minDist,iter));
%                 drawnow;
% %                 end
%             end
        
        % Genetic Algorithm Operators
        randomOrder = randperm(popSize); %popSize
        for p = 4:4:popSize%popSize
            
            rtes = pop(randomOrder(p-3:p),:);
            dists = totalDist(randomOrder(p-3:p));
            [ignore,idx] = min(dists); %#ok
            bestOf4Route = rtes(idx,:);
            
            for k = 1:4 % Mutate the Best to get Three New Routes
                tmpPop(k,:) = bestOf4Route;
                [~,sizeA]=size(tmpPop);
              
                 Index=[2:2:sizeA;(1:2:sizeA)+2]';
                 switch k
                    case 2 
                    
                        if p==4 && (m/2)==0
                        tmpPop1=zeros(4,8);    
                            % Cycle
                        tmpPop1(k,1:m)=tmpPop(1,1:m);
                        tmpPop(k,1:m) = tmpPop(1,m+1:end);
                        tmpPop(k,m+1:end) = tmpPop1(k,1:m);
                        
                        else
                         tmpPop2=tmpPop(k,:); 
          passenger_pick_drop = Index(datasample(randperm((sizeA/2),1),1),:); 
           tmpPop2(logical(sum( tmpPop2==passenger_pick_drop'))) = [];                     
                        insertion_indexes = sort(randperm(sizeA,2));
                      tmpPop2 = [tmpPop2(1:insertion_indexes(1)-1) passenger_pick_drop(1) tmpPop2(insertion_indexes(1):end)];
                       tmpPop(k,:) = [tmpPop2(1:insertion_indexes(2)-1) passenger_pick_drop(2) tmpPop2(insertion_indexes(2):end)]; 
                        
                        end
                   case 3
                        tmpPop2=tmpPop(k,:); 
                       
          passenger_pick_drop = Index(datasample(randperm((sizeA/2),1),1),:); 
           tmpPop2(logical(sum( tmpPop2==passenger_pick_drop'))) = [];                     
                        insertion_indexes = sort(randperm(sizeA,2));
                      tmpPop2 = [tmpPop2(1:insertion_indexes(1)-1) passenger_pick_drop(1) tmpPop2(insertion_indexes(1):end)];
                       tmpPop(k,:) = [tmpPop2(1:insertion_indexes(2)-1) passenger_pick_drop(2) tmpPop2(insertion_indexes(2):end)]; 
                      
                        
                        case 4
                             tmpPop2=tmpPop(k,:); 
          passenger_pick_drop = Index(datasample(randperm((sizeA/2),1),1),:); 
           tmpPop2(logical(sum( tmpPop2==passenger_pick_drop'))) = [];                     
                        insertion_indexes = sort(randperm(sizeA,2));
                      tmpPop2 = [tmpPop2(1:insertion_indexes(1)-1) passenger_pick_drop(1) tmpPop2(insertion_indexes(1):end)];
                       tmpPop(k,:) = [tmpPop2(1:insertion_indexes(2)-1) passenger_pick_drop(2) tmpPop2(insertion_indexes(2):end)]; 

                    otherwise % Do Nothing
                end
            end
            newPop(p-3:p,:) = tmpPop;
        end
        pop = newPop;
        
        % Update the waitbar
        if showWaitbar && ~mod(iter,ceil(numIter/325))
            waitbar(iter/numIter,hWait);
        end
        
    end
    if showWaitbar
        close(hWait);
    end
    
   %                         index=sort(reshape(randperm(8,8),[2,4]));
%                         tmpPop(k,index)=tmpPop(k,:);
                        
                        
                        
                        % insert
%                          tmpPop2(1,:) = tmpPop(k,1:end); 
%                           l= randi(7,1,1); 
%                          if l/2~=0
%                          tmpPop2(2,[I J])=tmpPop(k,[l l+1]);
%                          tmpPop2(1,[l l+1])=0;
%                          else 
%                          tmpPop2(2,[I J])=tmpPop(k,[l+1 l+2]);
%                          tmpPop2(1,[l+1 l+2])=0;
%                          end
%                          for i=1:8
%                              if tmpPop2(2,i)==0 && i~=I&&J && tmpPop2(1,i)~=0
%                                  tmpPop2(2,i)=tmpPop2(1,i);
%                              else tmpPop2(1,i)~=0
%                                   tmpPop2(2,i)=tmpPop2(1,i);
%                              end
%                          end 
     % Swap
%                         routeInsertionPoints = sort(ceil((n-1)*rand(1,2)));
%             I = routeInsertionPoints(1);
%             J = routeInsertionPoints(2);
%                         if I<J-1
%                         tmpPop1(k,[1 2]) = tmpPop(k,[I+1 J-1]); 
%                         if tmpPop(k,[I])<tmpPop(k,[I+1]) && tmpPop(k,I)<tmpPop1(k,1)
%                         tmpPop(k,[I I+1]) = tmpPop(k,[J I]);
%                         tmpPop(k,[J]) = tmpPop1(k,1);
%                         else 
%                         tmpPop(k,[I J-1]) = tmpPop(k,[J I]);
%                         tmpPop(k,[J]) = tmpPop1(k,2);  
%                         end
%                         else
%                             
%                             if I==1
%                              I=I+1;
%                             else 
%                         tmpPop1(k,[1 2]) = tmpPop(k,[I-1 J-1]);     
%                         tmpPop(k,[J-1 I-1]) = tmpPop(k,[J I]);
%                         tmpPop(k,[I J]) = tmpPop1(k,[1 2]); 
%                             end
                          
%                         end
%     if showResult
%         % Plots the GA Results
%         figure('Name','TSP_GA | Results','Numbertitle','off');
%         subplot(2,2,1);
%         pclr = ~get(0,'DefaultAxesColor');
%         if dims > 2, plot3(xy(:,1),xy(:,2),xy(:,3),'.','Color',pclr);
%         else plot(xy(:,1),xy(:,2),'.','Color',pclr); end
%         title('City Locations');
%        
%         subplot(2,2,2);
%         imagesc(dmat(optRoute,optRoute));
%         title('Distance Matrix');
%         subplot(2,2,3);
%         rte = optRoute([1:n 1]);
%         if dims > 2, plot3(xy(rte,1),xy(rte,2),xy(rte,3),'r.-');
%         else plot(xy(rte,1),xy(rte,2),'r.-'); end
%         title(sprintf('Total Distance = %1.4f',minDist));
%         subplot(2,2,4);
%         plot(distHistory,'b','LineWidth',2);
%         title('Best Solution History');
%         set(gca,'XLim',[0 numIter+1],'YLim',[0 1.1*max([1 distHistory])]);
%     end
   
    % Return Output
    if nargout
        resultStruct = struct( ...
            'xy',          xy, ...
            'dmat',        dmat, ...
            'popSize',     popSize, ...
            'numIter',     numIter, ...
            'showProg',    showProg, ...
            'showResult',  showResult, ...
            'showWaitbar', showWaitbar, ...
            'optRoute',    optRoute, ...
            'rte', rte,...
            'minDist',     minDist);
        
        varargout = {resultStruct};
      
    end
    
end
% Subfunction to override the default configuration with user inputs
function config = get_config(defaultConfig,userConfig)
    
    % Initialize the configuration structure as the default
    config = defaultConfig;
    
    % Extract the field names of the default configuration structure
    defaultFields = fieldnames(defaultConfig);
    
    % Extract the field names of the user configuration structure
    userFields = fieldnames(userConfig);
    nUserFields = length(userFields);
    
    % Override any default configuration fields with user values
    for i = 1:nUserFields
        userField = userFields{i};
        isField = strcmpi(defaultFields,userField);
        if nnz(isField) == 1
            thisField = defaultFields{isField};
            config.(thisField) = userConfig.(userField);
        end
    end
end
end