%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%  Code modified by Francisco Vasconcelos from
%%%%
%%%%  Drone class, feel free to add functionality as you see fit
%%%%  Author: Daniel Butters
%%%%  Date: 16/11/17
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
classdef Drone < handle
    properties (Constant)
        %width, length, height offset between centre and rotors
        body = [0.6 0.6 0.0];
        
        %time interval for simulation (seconds)
        time_interval = 0.02;
        
        % size of floating window that follows drone
        axis_size = 2.;
        
        %colours of each component of drone model
        colours = [[.8 .3 .1];[.2 .2 .5];[.8 .1 .3];[.9 .6 .8];[.9 .2 .4]];

        %Kinematics constants
        m=0.2;
        I=[1,0,0;0,1,0;0,0,0.5];
        g=9.8;
        kd=0.1;
        k=1;
        L=0.2;
        b=0.1;
        
        
        %Follows the drone within the figure
        %Don't use if you are simulating more than one drone!
        %Switch to false to see the overall world view
        drone_follow = false;
    end
    properties
        %axis to draw on
        axis
        
        %length of one side of the flight arena
        spaceDim
        
        %limits of flight arena
        spaceLimits
        
        %drone position
        pos

        % rotaion matrix
        R
        
        %Simulation time
        time

        %number of drones
        num_drones

        %kinematics parameters
        theta;
        thetadot;
        xdot;
        posRecord
        orienRecord
        omega;
    end
    methods
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %INSTANTIATION OF CLASS
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function obj = Drone(axis, spaceDim, num_drones)
            if nargin > 1
                obj.axis = axis;
                
                obj.spaceDim = spaceDim;
                
                obj.spaceLimits = [(-spaceDim/2)+10 (spaceDim/2)-10 (-spaceDim/2)+10 (spaceDim/2)-10 10 spaceDim-10];
                
                % 1.a. set the starting point at the altitude of 5
                obj.pos = [0;0;5];
                
                obj.R = [1,0,0;0,1,0;0,0,1];
                
                obj.time = 0;
                
                obj.num_drones = num_drones;
                obj.xdot=zeros(3,1);
                obj.theta=zeros(3,1);
                
                obj.thetadot=zeros(3,1);
                obj.omega=zeros(3,1);
            else
                error('Drone not initialised correctly')
            end
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %DRAWING OF DRONE TO FIGURE
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function draw(obj)
            %how big should the moving window be
            cL = obj.axis_size;
            
            %set to false if you want to see world view
            %if(obj.drone_follow)
            %    axis([obj.pos(1)-cL obj.pos(1)+cL obj.pos(2)-cL obj.pos(2)+cL obj.pos(3)-cL obj.pos(3)+cL]);
            %end
            
            %create middle sphere
            [X Y Z] = sphere(8);
            %[X Y Z] = (obj.body(1)/5.).*[X Y Z];
            X = (obj.body(1)/5.).*X + obj.pos(1);
            Y = (obj.body(1)/5.).*Y + obj.pos(2);
            Z = (obj.body(1)/5.).*Z + obj.pos(3);
            s = surf(obj.axis,X,Y,Z);
            set(s,'edgecolor','none','facecolor',obj.colours(1,:));
            
            %create side spheres
            %front, right, back, left
            hOff = obj.body(3)/2;
            Lx = obj.body(1)/2;
            Ly = obj.body(2)/2;
            rotorsPosBody = [...
                0    Ly    0    -Ly;
                Lx    0    -Lx   0;
                hOff hOff hOff hOff];
            rotorsPosInertial = zeros(3,4);
            for i = 1:4
                rotorPosBody = rotorsPosBody(:,i);
                rotorsPosInertial(:,i) = bodyToInertial(obj,rotorPosBody);
                [X Y Z] = sphere(8);
                X = (obj.body(1)/8.).*X + obj.pos(1) + rotorsPosInertial(1,i);
                Y = (obj.body(1)/8.).*Y + obj.pos(2) + rotorsPosInertial(2,i);
                Z = (obj.body(1)/8.).*Z + obj.pos(3) + rotorsPosInertial(3,i);
                s = surf(obj.axis,X,Y,Z);
                set(s,'edgecolor','none','facecolor',obj.colours(i+1,:));
            end
            obj.axis.Title.String = ['Sim Time = ',num2str(obj.time,'%f'),' seconds'];
        end
        
        function vectorInertial = bodyToInertial(obj, vectorBody)
            vectorInertial = obj.R*vectorBody;
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % SIMULATION FUNCTIONS(Callable Function)
        % Modified by Jian
        % 15/12/2022
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        %demo (not useful) code to show varying position and rotation
        %replace with your own functions!
      
        
        
        function update(obj)
            %update simulation time
            obj.time = obj.time + obj.time_interval;
            
            %change position and orientation of drone
           %% input
            % 1.b.
            num_gamma=obj.m*obj.g/4/obj.k;
            inputs=[num_gamma;num_gamma;num_gamma;num_gamma];
            
            % 1.c.
            if (obj.time>2)&&(obj.time<=4)
                inputs=inputs*1.15;
            end
            if (obj.time>4)&&(obj.time<=8)
                inputs(4)=0;
            end
            %% update kinematicss
            kinematics(obj,inputs);
            %draw drone on figure
            draw(obj);
        end
        

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Tool Functions
        % Modified by Jian
        % 15/12/2022
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function T=thrust(~,inputs,k)
        T=[0;0;k*sum(inputs)];
        end

        function tau=torques(~,inputs,L,b,k)
            % Inputs are values for omega^2
            tau = [L * k * (inputs(1) - inputs(3))
                   L * k * (inputs(2) - inputs(4))
                   b * (inputs(1) - inputs(2) + inputs(3) - inputs(4))
                   ];
        end

        function a= acceleration(obj,inputs,angles,xdot,m,g,k,kd)
            gravity=[0;0;-g];
            obj.R=rotation(obj,angles);
            T=obj.R*thrust(obj,inputs,k);
            Fd=-kd*xdot;
            a=gravity+1/m*T+Fd;
        end

        function omegadot=angular_acceleration(obj,inputs,omega,I,L,b,k)
            tau=torques(obj,inputs,L,b,k);
            omegadot=inv(I)*(tau-cross(omega,I*omega)); 
        end

        function R=rotation(~,Theta)

            Rx=[1,0          ,0           ;
                0,cos(Theta(1)),-sin(Theta(1));
                0,sin(Theta(1)),cos(Theta(1))];
            
            Ry=[cos(Theta(2)),0,sin(Theta(2));
                0          ,1,0          ;
                -sin(Theta(2)),0,cos(Theta(2))];
            Rz=[cos(Theta(3)),-sin(Theta(3)),0;
                sin(Theta(3)),cos(Theta(3)),0;
                0,0,1];
            R=Rz*Ry*Rx;
        end

        function omega=thetadot2omega(~,thetadot,theta)
                 omega=[1, 0             , -sin(theta(2));
                        0, cos(theta(1)) , cos(theta(2))*sin(theta(1));
                        0, -sin(theta(1)),cos(theta(2))*cos(theta(1))]*thetadot;
        end
        
        function thetadot=omega2thetadot(~,omega,theta)
            thetadot  =  inv([1 , 0             , -sin(theta(2)) ; 
                              0 , cos(theta(1)) , cos(theta(2))*sin(theta(1));
                              0 , -sin(theta(1)), cos(theta(2))*cos(theta(1))  ] )*omega ;
        
        end
        
        function kinematics(obj,inputs)
        %% kinematics
            dt=obj.time_interval;
            obj.omega=thetadot2omega(obj,obj.thetadot,obj.theta);
            %Compute linear and angular accelerations
            a=acceleration(obj,inputs,obj.theta,obj.xdot,obj.m,obj.g,obj.k,obj.kd);
            omegadot=angular_acceleration(obj,inputs,obj.omega,obj.I,obj.L,obj.b,obj.k);       
            obj.omega=obj.omega+dt*omegadot;
            obj.thetadot=omega2thetadot(obj,obj.omega,obj.theta);
            obj.theta=obj.theta+dt*obj.thetadot;
            obj.xdot=obj.xdot+dt*a;
            obj.pos=obj.pos+dt*obj.xdot;


            obj.posRecord = [obj.posRecord , obj.pos];            
            obj.orienRecord = [obj.orienRecord , obj.theta];
        end
      
    end
end
