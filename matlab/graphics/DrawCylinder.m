function varargout = DrawCylinder(varargin)
%Draw new 3D Cylinder with filled top and bottom, or update position and
%orientation of existing cylinder. The number of sides of the cylinder
%defines the roundness of the cylinder. Use nSides = 4 to make a
%rectangular box.
%   Plot new Cylinder: CylinderData = DrawCylinder([x_length; y_width; z_height], nSides, HomogenousMatrix, color, alpha, LidDirection) 
%   Update existing Cylinder: DrawCylinder(CylinderData, HomogenousMatrix)
%   Delete existing Cylinder: DrawCylinder(CylinderData)

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Process function arguments
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    action_update_Cylinder = 1;
    action_new_Cylinder    = 2;
    action_delete_Cylinder = 3;

    if(nargin == 2)
        % 2 input arguments means we update the existing cylinder
        action           = action_update_Cylinder;
        CylinderData     = varargin{1};
        HomogenousMatrix = varargin{2};
    elseif(nargin == 1)
        % 1 input arguments means we delete the existing cylinder
        action           = action_delete_Cylinder;
        CylinderData     = varargin{1};
    elseif(nargin == 6)
        % 5 input arguments means we create a new cylinder
        action = action_new_Cylinder;
        CylinderData.Size  = varargin{1};
        CylinderData.nSides= varargin{2};
        HomogenousMatrix   = varargin{3};
        CylinderData.Color = varargin{4};
        CylinderData.Alpha = varargin{5};
        CylinderData.LidDirection = varargin{6};
    elseif(nargin == 0)
        % 0 input arguments means we create a new cylinder with dummy dimensions
        action = action_new_Cylinder;
        CylinderData.Size = [3; 2; 1]; % Create a cylinder with dimensions 3x2x1 meter
        CylinderData.nSides = 8;       % Base of cylinder has 4 sides (so we are actually making a rectanglular bar)
        HomogenousMatrix = eye(4);     % The orientation is simply identity: no displacement and rotation
        CylinderData.Color =  [0    0.4470    0.7410];  %The color is blue.     
        CylinderData.Alpha =  1;  % Fully opaque  

    else
        error('Incorrect number of arguments')
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Make new cylinder (if this action was selected)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%              
    if(action == action_new_Cylinder)
            
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Make local vertex points
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
        
        % Check number of sides
        nSides = CylinderData.nSides;
        if(nSides <= 0 || mod(nSides,4)~=0)
            error('Number of sides must be postive multiple of 4')
        end

        %Angle from the center to vertex point, equally spread
        firstangle = (2*pi/nSides)/2; %Half the angle between two vertices
        lastangle  = -firstangle+2*pi;%Negative angle of the first angle
        angles     = linspace(firstangle,lastangle,nSides);

        %Unscaled local x,y,z coordinates of vertices
        x = cos(angles);
        y = sin(angles);
        z = ones(size(x));

        %Define x and y scaling of local point based on outer dimensions
        xscale = CylinderData.Size(1)/(max(x)-min(x));
        yscale = CylinderData.Size(2)/(max(y)-min(y));
        zscale = CylinderData.Size(3)/2;

        %Define the local vertex point as homogenous coordinates
        CylinderData.LocalVertices = [x*xscale x*xscale; y*yscale y*yscale; z*zscale -z*zscale; ones(1,length(angles)*2)];

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Transform local vertex points to global frame
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%         
        
        if(strcmp(CylinderData.LidDirection,'x'))
            CylinderData.PreRotation = [0 0 -1 0; 0 1 0 0; 1 0 0 0; 0 0 0 1];
        elseif(strcmp(CylinderData.LidDirection,'y'))
            CylinderData.PreRotation = [1 0 0 0; 0 0 1 0; 0 -1 0 0; 0 0 0 1];
        else
            CylinderData.PreRotation = eye(4);
        end
        
        
        
        vertices_global = CylinderData.PreRotation*HomogenousMatrix*CylinderData.LocalVertices;          
        vertices_globalxyz = vertices_global(1:3,:)';

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Associate faces with vertex indices
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
        faces = zeros(nSides+2,nSides);
        for i=1:nSides-1
            faces(i,:) = [i,i+1,i+nSides+1,i+nSides repmat(i+nSides,[1 nSides-4])];
        end
        faces(nSides  ,:) = [nSides,1,nSides+1,nSides*2 repmat(nSides*2,[1 nSides-4])];
        faces(nSides+1,:) = 1:nSides;
        faces(nSides+2,:) = nSides+1:nSides*2; 
        
        CylinderData.Faces = faces;
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Plot the global vertex points as patches
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
        CylinderData.hPatch = patch('Faces',CylinderData.Faces,'Vertices',vertices_globalxyz,'FaceColor',CylinderData.Color,'FaceAlpha',CylinderData.Alpha);

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Output the structure CylinderData containing all data to move this cylinder later
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%             
        varargout{1} = CylinderData;      
       
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Update existing cylinder (if this action was selected)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%       
    elseif(action == action_update_Cylinder)
       
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Transform local vertex points to global frame
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%         

        vertices_global = HomogenousMatrix*CylinderData.PreRotation*CylinderData.LocalVertices; 
        vertices_globalxyz = vertices_global(1:3,:)';
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Update vertices in all of the patches
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%             

        set(CylinderData.hPatch,'Vertices',vertices_globalxyz);
        
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Delete cylinder (if this action was selected)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    elseif(action == action_delete_Cylinder)
        % Simply delete all the patches
        for h = CylinderData.hPatches;
            delete(h);
        end        
    else
        error('invalid action')
    end
        
end