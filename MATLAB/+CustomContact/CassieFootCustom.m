%% Function: CassieFootCustom
%
% Description: This function applies a holonomic constraint to a robot with
%   line contacts along the X-axis. Input for foot controls which leg it is
%   applied to, and the addZMP option determines whether a unilateral
%   constraint is added to bound the ZMP. 
%
% Author: Jenna Reher, jreher@caltech.edu
% ________________________________________

function [ domain ] = CassieFootCustom( domain, foot, addZMP )
    %% Setup
    if strcmp(foot, 'Left')
        contactPoint = domain.ContactPoints.LeftSole;
    elseif strcmp(foot, 'Right')
        contactPoint = domain.ContactPoints.RightSole;
    else
        error('Not a valid leg of Cassie! Say Left or Right.');
    end

    contact = ToContactFrame(contactPoint, 'PointContactWithFriction');
    
    % Friction Cone Information
    fric_coef.mu = 0.6;
    fric_coef.gamma = 25;%fric_coef.mu * norm(domain.p_toe - domain.p_midfoot);
    
    % ZMP Information
    geometry.la = 0.5 * norm(domain.p_toe - domain.p_midfoot);
    geometry.lb = 0.5 * norm(domain.p_midfoot - domain.p_heel);

    % Create the wrench base
    ref = eye(3);
    I = eye(6);
    planar = false;
    if strcmp(domain.Joints(3).Name, 'BaseRotY')
        planar = true;
        % The model is planar
        wrenchBase = I(:,[1,3,5]); % x, z, pitch
    else
        % We are full 3d
        wrenchBase = I(:,[1,2,3,5,6]); % x, y, z, pitch, yaw
    end
    
    G = [eye(3),     zeros(3,3); 
         zeros(3,3), ref]...
         * wrenchBase;
    
    % compute the spatial position (cartesian position + Euler angles)
    pos = getCartesianPosition(domain, contact);
    rpy = getRelativeEulerAngles(domain, contact, ref);
       
    h = transpose([pos, rpy]); %effectively as transpose
    % extract the contrained elements
    constr =  G' * h;
    % compute the body jacobian
    jac = getBodyJacobian(domain, contact);
    constr_jac = wrenchBase' * jac;
    % compute the spatial jacobian for Cartesian positions
    constr_jac(1:3,:) = jacobian(constr(1:3), domain.States.x);
    
    % label for the holonomic constraint
    label_full = cellfun(@(x)[contact.Name,x],...
        {'PosX','PosY','PosZ','Roll','Pitch','Yaw'},'UniformOutput',false);
    for i=size(wrenchBase,2):-1:1
        label{i} = label_full{find(wrenchBase(:,i))};         %#ok<FNDSB>
    end
    
    % create a holonomic constraint object
    contact_constr = HolonomicConstraint(domain,...
                                         constr, contact.Name,...
                                         'Jacobian',constr_jac,...
                                         'ConstrLabel',{label},...
                                         'DerivativeOrder',2);
    % add as a set of holonomic constraints
    domain = addHolonomicConstraint(domain, contact_constr);
    
    % the contact wrench input vector
    f_name = contact_constr.InputName;
    f = domain.Inputs.ConstraintWrench.(f_name);
    
    %% Friction Cone
    % get the friction cone constraint
    mu = SymVariable('mu');
    gamma = SymVariable('gamma');
    fun_name = ['u_friction_cone_', domain.Name];
    
    if planar
        % x, y, z, pitch
        constr = [f(2); % fz >= 0
                  f(1) + (mu/sqrt(2))*f(2);  % -mu/sqrt(2) * fz < fx
                 -f(1) + (mu/sqrt(2))*f(2)]; % fx < mu/sqrt(2) * fz

        % create a symbolic function object
        f_constr = SymFunction(fun_name,...
            constr,{f},{mu});

        % create the label text
        label = {'normal_force';
                 'friction_x_pos';
                 'friction_x_neg'};
             
        % validate the provided static friction coefficient
        validateattributes(fric_coef.mu,{'double'},...
            {'scalar','real','>=',0},...
            'ContactFrame.getFrictionCone','mu');
        auxdata = fric_coef.mu;
    else
        % x, y, z, yaw
        constr = [f(3); % fz >= 0
                  f(1) + (mu/sqrt(2))*f(3);  % -mu/sqrt(2) * fz < fx
                 -f(1) + (mu/sqrt(2))*f(3);  % fx < mu/sqrt(2) * fz
                  f(2) + (mu/sqrt(2))*f(3);  % -mu/sqrt(2) * fz < fu
                 -f(2) + (mu/sqrt(2))*f(3);  % fy < mu/sqrt(2) * fz
                  f(5) + gamma * f(3);       % -gamma * fz < wz
                 -f(5) + gamma * f(3)];      % wz < gamma * fz

        % create a symbolic function object
        f_constr = SymFunction(fun_name,...
            constr,{f},{[mu;gamma]});

        % create the label text
        label = {'normal_force';
                 'friction_x_pos';
                 'friction_x_neg';
                 'friction_y_pos';
                 'friction_y_neg';
                 'tor_friction_neg';
                 'tor_friction_pos'};

        % validate the provided static friction coefficient
        validateattributes(fric_coef.mu,{'double'},...
            {'scalar','real','>=',0},...
            'ContactFrame.getFrictionCone','mu');

        % validate the provided torsional friction coefficient
        validateattributes(fric_coef.gamma,{'double'},...
            {'scalar','real','>=',0},...
            'ContactFrame.getFrictionCone','gamma');
        auxdata = [fric_coef.mu; fric_coef.gamma];
    end
    
    % create an unilateral constraint object
    fc_cstr = UnilateralConstraint(domain, f_constr,...
                                    ['fc' contact.Name], f_name, ...
                                    'ConstrLabel',{label(:)'},...
                                    'AuxData',auxdata);
    % add as a set of unilateral constraitns
    domain = addUnilateralConstraint(domain, fc_cstr);
    
    %% Add ZMP
    if addZMP
        la = SymVariable('gla');
        lb = SymVariable('glb');
        fun_name = ['u_zmp_', domain.Name];

        if planar
            % x, y, z, roll, yaw
            zmp = [la*f(2) - f(3);  % la*fz > my
                   lb*f(2) + f(3)]; % my > -lb*fz
        else
            % x, y, z, roll, yaw
            zmp = [la*f(3) - f(4);  % la*fz > my
                   lb*f(3) + f(4)]; % my > -lb*fz
        end

        % create a symbolic function object
        f_constr = SymFunction(fun_name, zmp, {f}, {[la;lb]});

        % create the label text
        label = {'pitch_pos';
                 'pitch_neg'};

        % validate the provided static friction coefficient
        validateattributes(geometry.la,{'double'},...
            {'scalar','real','>=',0},...
            'ContactFrame.getZMPConstraint','la');
        validateattributes(geometry.lb,{'double'},...
            {'scalar','real','>=',0},...
            'ContactFrame.getZMPConstraint','lb');
        auxdata = [geometry.la;geometry.lb];

        zmp_cstr = UnilateralConstraint(domain, f_constr,...
                                        ['zmp' contact.Name], f_name, ...
                                        'ConstrLabel',{label(:)'},...
                                        'AuxData',auxdata);
        % add as a set of unilateral constraitns
        domain = addUnilateralConstraint(domain, zmp_cstr);
    end
    
    
    
    
end

