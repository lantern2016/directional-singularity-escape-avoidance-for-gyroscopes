function Animate(time,Y, bodies, plotConfig)
% Animation of the Simulation Result. ADD USAGE INFO. EXPLAIN 2 time scales

    %%% Diplay statistics
    disp(['Starting animation at ' num2str(plotConfig.playBackSpeed) 'x real time'])
    
    %%% Create the figure, with function that handles it closure.
    AnimationWindow = figure('units','normalized','position',[0.52 0.05 0.48 0.9]);hold on;
    AnimationWindow.CloseRequestFcn = @close_animation;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% Monitor Close Button to set Exit flag
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    exit = 0;    
    function close_animation(source,event)
        exit = 1;
        delete(gcbf);
    end  
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% Pause Button Handling
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Create the button
    btnPause = uicontrol('Style', 'pushbutton', 'String', 'Continue',...
        'Position', [20 20 80 20],...
        'Callback', @pause_animation);  
    
    % Flag that monitors whether the simulation is paused (1) or not
    paused = 1;
    
    % Time that the simulation last started or continued
    displayStartTime = 0;
    
    % Pause event handler
    function pause_animation(source,event)
        if(paused)
            % If currently paused, toggle to not paused
            paused = 0;
            source.String = 'Pause';
            tic;
        else
            % If currently not paused, toggle to paused
            displayStartTime = displayStartTime + toc;
            paused = 1;
            source.String = 'Continue';
        end     
    end       
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% Reset Button Handling
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    
    % Create push button
    btnRestart = uicontrol('Style', 'pushbutton', 'String', 'Restart',...
        'Position', [120 20 80 20],...
        'Callback', @restart_animation);      
 
    % Flag that indicates whether the plot has been displayed at least once
    % (1) or not yet (0). This allows us to make one initial plot, even if
    % the simulation is paused.
    firstShowCompleted = 0;

    % Reset the animation, by simply setting the current display time to 0.
    function restart_animation(source,event)
            displayStartTime = 0;
            firstShowCompleted = 0;
            tic;
    end  

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% Animation setup
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    
    %Initial state
    y0 = Y(:,1);
    
    %Create the bodies in the initial position
    nBodies = length(bodies);
    for k = 1:nBodies
        hBody{k}  = DrawCylinder(bodies{k}.dimensions, bodies{k}.sides, bodies{k}.H(y0), bodies{k}.color(1:3), bodies{k}.color(4), bodies{k}.up);
    end

    %%% Display timer in the plot title
    hTime = title('Time: 0.00 s');
    
    % Axes settings
    axis equal
    view(125,30)
    
    xlim(plotConfig.axisLimits{1})
    ylim(plotConfig.axisLimits{2})
    zlim(plotConfig.axisLimits{3})

    xlabel('x')
    ylabel('y')
    zlabel('z')
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% Main animation loop: Repeatedly update the XYZ data
    %%% of the existing plot elements, based on the state value
    %%% at the current time.
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
    
    % Animation Loop
    nSteps        = length(time);
    framerate     = 30;
    tic;

    % This function interpolates a function result between two time steps
    interpFunc = @(hFunc,Yprev,Ynext,d) hFunc(Yprev) + d*(hFunc(Ynext)-hFunc(Yprev));

    % Keep updating the plots until an exit condition occurs, such as the
    % closing of the plot.
    while(~exit)
        
        % Do nothing if we don't need to plot anything, such as when paused
        if (paused && firstShowCompleted)
            pause(0.05)
        else
            % If this is the first plot, store that it has been completed.
            if(~firstShowCompleted)
                firstShowCompleted = 1;
                tic;
            end
            
            % Display time (real elapsed animation time) and according simulation time, given the play
            % back speed.
            displayTime = toc + displayStartTime;
            simulationTime = displayTime*plotConfig.playBackSpeed;
        
            % Find the simulation index corresponding to time
            nextSimulationTimeIndex = find(time>=simulationTime,1);

            % Check whether there is a next simulation index at all
            if(isempty(nextSimulationTimeIndex) || nextSimulationTimeIndex > nSteps)
                pause(0.05)
                continue;
            end
            
            % The next simulation time index must be at least 2
            if(nextSimulationTimeIndex == 1); nextSimulationTimeIndex = 2; end
                        
            % Find the next and previous state vector
            Ynext = Y(:,nextSimulationTimeIndex  );
            Yprev = Y(:,nextSimulationTimeIndex-1);
            tnext = time(nextSimulationTimeIndex);
            tprev = time(nextSimulationTimeIndex-1);

            % The ratio within the two time steps
            d = (simulationTime-tprev)/(tnext-tprev); 

            % Update the timer
            try 
                hTime.String = ['Time: ' num2str(simulationTime,'%.2f') ' s'];
            catch
                
            end
            
            % Update the rigid bodies
            for k = 1:nBodies
                    
                try
                    % For each body, retrieve its homogenous transformation
                    % matrix
                    Hnow = interpFunc(bodies{k}.H,Yprev,Ynext,d);
                    
                    % The H matrix should always have determinant 1. If the
                    % H matrix rapidly changes within two time intervals,
                    % this typically indicates that the sampling time is
                    % too low, or that a singularity has occured, causing
                    % infinite growth of one or more signals.
                    if(det(Hnow) < 0.9)
                        error(['Meaningless Transformation Matrix (det = ' num2str(det(Hnow)) ') for body ' num2str(k) '. Is the simulation sample rate high enough?'])
                    end                    

                    % Use the transformation matrix to update the body
                    % location
                    DrawCylinder(hBody{k},Hnow);
                    
                catch 
                    % If the figure is closed or plotting is not possible
                    % for some other reason, the animation is closed. 
                    disp('Animation was cancelled.')
                    exit = 1;
                    break;
                end

            end

            % Pause to make the plot update rate match the desired
            % framerate
            pause(1/framerate)

            % Quit the loop if requested by the user (by closing the
            % figure) or when a plot error occurs
            if(exit)
                break;
            end

        end    
        
    end
    
    
end

