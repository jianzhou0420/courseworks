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
        drone_follow = true;
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
        posDot;
        omega;
        sys;
        A;
        B;
        state_current;
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
                obj.posDot=zeros(3,1);
                obj.state_current=[obj.pos;obj.posDot;obj.theta;obj.omega];
                obj.A=zeros(12,12);
                obj.B=zeros(12,4);
                
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
        function obj = change_pos_and_orientation(obj)
           
            obj.pos
        end
        
        
        function update(obj)
            %update simulation time
            obj.time = obj.time + obj.time_interval;
            %% input
            % 2.b.
            num_gamma=obj.m*obj.g/4/obj.k;
            input=[num_gamma;num_gamma;num_gamma;num_gamma];
            
            % 2.c.
            if (obj.time>2)&&(obj.time<=4)
                input=input*1.2;
            end

            if (obj.time>4)&&(obj.time<=8)
                input(4)=0;
            end

            %% kinematics
            % update
            obj.sys=getLTIPara(obj,input);
            obj.A=obj.sys.A;
            obj.B=obj.sys.B;
            obj.state_current=obj.A*(obj.state_current-zeros(12,1))...
                +obj.B*(input-[num_gamma;num_gamma;num_gamma;num_gamma]);
            
            % data process
            obj.pos=obj.state_current(1:3);
            obj.theta=obj.state_current(7:9);
            obj.R=rotation(obj,obj.theta);
            obj.posRecord = [obj.posRecord , obj.pos];            
            obj.orienRecord = [obj.orienRecord , obj.theta];
            
            %draw drone on figure
            draw(obj);
        end
        

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Tool Functions
        % Modified by Jian
        % 15/12/2022
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function sys=getLTIPara(obj,input)
            syms Theta [3 1]
            syms Omega [3 1] 
            syms X [4 1]
            syms Xdot [4 1] 
            syms u [4 1] 
            syms Pos [3 1] 
            syms Vel [3 1]
            % Define some constants
            Rx=[1,0          ,0           ;
                0,cos(Theta1),-sin(Theta1);
                0,sin(Theta1),cos(Theta1)];
            
            Ry=[cos(Theta2),0,sin(Theta2);
                0          ,1,0          ;
                -sin(Theta2),0,cos(Theta2)];
            Rz=[cos(Theta3),-sin(Theta3),0;
                sin(Theta3),cos(Theta3),0;
                0,0,1];
            R=Rz*Ry*Rx;
            % Define the LTI system
            X1 = Pos ;
            X2 = Vel; 
            X3 = Theta;
            X4 = Omega;
            
            Xdot1=Vel;
            Xdot2=[0;0;-obj.g]+1/obj.m*R*obj.k*[0;0;u1+u2+u3+u4]-obj.kd/obj.m*Vel;
%             Xdot2 = [0;0;-obj.g] + 1/obj.m  *(Rz*Ry*Rx)* obj.k*[0;0;(u1+u2+u3+u4)] - 1/obj.m*obj.kd* Vel;
            Xdot3=[1,0,-sin(Theta2); ...
                       0,cos(Theta1),cos(Theta2)*sin(Theta1); ...
                       0, -sin(Theta1),cos(Theta2)*cos(Theta1)]\Omega;
%             Xdot4=inv(obj.I)*[obj.L*obj.k,0,-obj.L*obj.k,0;
%                           0,obj.L*obj.k,0,-obj.L*obj.k;
%                           obj.b,-obj.b,obj.b,-obj.b]*[u1;u2;u3;u4]-inv(obj.I)*(cross(Omega,obj.I*Omega));
%             Xdot4 = inv(obj.I)*([obj.L*obj.k*(u1-u3); ...
%                                 obj.L*obj.k*(u2-u4); ...
%                                 obj.b*(u1-u2+u3-u4)]-cross(Omega,obj.I*Omega));
            Xdot4 = [(obj.L * obj.k * (u1 - u3))/(obj.I(1,1)); 
                    (obj.L * obj.k * (u2 - u4))/(obj.I(2,2));
                    (obj.b * (u1 - u2 + u3 - u4))/(obj.I(3,3))] ... 
                    - [(obj.I(2,2) - obj.I(3,3))/obj.I(1,1) * Omega2*Omega3;
                    (obj.I(3,3) - obj.I(1,1))/obj.I(2,2) * Omega1*Omega3; 
                    (obj.I(1,1)-obj.I(2,2))/obj.I(3,3) * Omega1*Omega2];


            A=jacobian([Xdot1;Xdot2;Xdot3;Xdot4],[Pos;Vel;Theta;Omega]);
            B=jacobian([Xdot1;Xdot2;Xdot3;Xdot4],[u1;u2;u3;u4]);


            A=subs(A,u,input);
            A=subs(A,Pos,obj.state_current(1:3));
            A=subs(A,Vel,obj.state_current(4:6));
            A=subs(A,Omega,obj.state_current(7:9));
            A=subs(A,Theta,obj.state_current(10:12));
            


           
            B=subs(B,Pos,obj.state_current(1:3));
            B=subs(B,Vel,obj.state_current(4:6));
            B=subs(B,Omega,obj.state_current(7:9));
            B=subs(B,Theta,obj.state_current(10:12));
            
            
            
            A=double(A);
            B=double(B);
            C = eye(size(A));
            D = zeros(size(B));
            sys=c2d(ss(A,B,eye(12),zeros(12,4)),obj.time_interval,'zoh');

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
      
    end
end
