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

        %conroller constants
        tolerance=0.15;
        gamma_hover=0.49;
        
        
        
        
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
        posRecord;
        orienRecord;
        a;
       
        
        omega;
        % flags(1)=1 if the drone has successfully moved to (5,5,5)
        % flags(2)=1 if the froe has hovered at (5,5,5) for 5 seconds
        % flags(3)=1 if the drone has moved a circular trajectory
        % flags(4)=1 if the drone has landed
        flags=zeros(4,1);
        taskE_check_flag=0;

        %%
        current_step=0;
        timmer_in_use=0;
        
        ep1_last=0;
        ep2_last=0;
        ei2_last=0;

        %%
        ei1_last=0;
        T_B=[0;0;0];
        last_time=0;
        
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
                obj.pos = [0;0;0];
                
        
                obj.R = [1,0,0;0,1,0;0,0,1];
                
                obj.time = 0;
                
                obj.num_drones = num_drones;
                obj.xdot=zeros(3,1);
                obj.theta=zeros(3,1);
                
                obj.thetadot=zeros(3,1);
                obj.omega=zeros(3,1);
                obj.a=zeros(3,1);
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
      

        function inputs=controller(obj,pos)
            %% firstly, get pos_ref basing on current step
            if obj.flags==[0;0;0;0]
                pos_ref=taskABC(obj,pos);
            elseif obj.flags==[1;0;0;0]
                pos_ref=taskABC(obj,pos);
            elseif obj.flags==[1;1;0;0]
                pos_ref=taskD(obj);
            elseif obj.flags==[1;1;1;0]
                pos_ref=taskE(obj);
            else
                aaa=0;
                pos_ref=[5;5;0];
            end

            %% The first controller
            Kp1=0.06;
            Ki1=0.01;
            Kd1=1;

            ep1=pos_ref(3)-obj.pos(3);
            ei1=obj.ei1_last+ep1*obj.time_interval;
            ed1=(ep1-obj.ep1_last)/obj.time_interval;

            thrust_additional= Kp1*ep1 + Ki1*ei1 + Kd1*ed1;

            obj.ep1_last=ep1;
            obj.ei1_last=obj.ei1_last+ep1*obj.time_interval;

            Thrust_next=obj.m*obj.g/obj.k/cos(obj.theta(1))/...
                cos(obj.theta(2))+thrust_additional;

            %% The second controller
            pos_vector=pos_ref-obj.pos;
            
            if obj.a == zeros(3,1)
               a_Vector = [0;0;1];
            else
                a_Vector =[obj.a(1);obj.a(2);1];
            end 
            if obj.time==122
                rr=1;
            end
            rAngle = vrrotvec( a_Vector,pos_vector);
            

            Kp2=80;
            Ki2=6;
            Kd2=10;
            
            ep2=obj.theta;
            ei2=obj.ei2_last+obj.time_interval*ep2;
            ed2=(ep2-obj.ep2_last)/obj.time_interval;

            e=obj.I*(Kp2*ep2+Ki2*ei2+Kd2*ed2)-transpose(rAngle(1:3));
    
            obj.ei2_last=ei2;
            obj.ep2_last=ep2;
            
            
            inputs=error2inputs(obj,e,Thrust_next);
            
        end

        function pos_ref=taskABC(obj,pos)
            pos_ref=[5;5;5];
            % requirements check
            if all(pos>=5-obj.tolerance) && all(pos<=5+obj.tolerance)
                if obj.timmer_in_use==0 % means the first time we enter this if
                % initialize a timmer
                obj.timmer_in_use=1;
                obj.current_step=0;
                %set flag
                obj.flags(1)=1;
                end
                obj.current_step=obj.current_step+1;
                if obj.current_step>=250
                    obj.flags(2)=1;
                    obj.timmer_in_use=0;
                    
                end
  
            end

        end

        
        function pos_ref=taskD(obj)
        if obj.timmer_in_use==0
            % initialize a timmer
                obj.timmer_in_use=1;
                obj.current_step=0;
        end
        %suppose we use 80 seconds to finish the circular, thus 1000steps
        theta_circles=obj.current_step*pi/30;
        pos_ref=[2.5+2.5*cos(theta_circles);5+2.5*sin(theta_circles);5];
        diff=sqrt((obj.pos(1)-pos_ref(1))^2+(obj.pos(2)-pos_ref ...
            (2))^2);
        
        if all(obj.pos>=pos_ref-obj.tolerance) && all(obj.pos<=pos_ref+obj.tolerance)
            obj.current_step=obj.current_step+1;
        end
        if obj.current_step==60
            obj.flags(3)=1;
            obj.timmer_in_use=0;
            obj.current_step=0;
            pos_ref=[5;5;5];
        end
        end
      
        function pos_ref=taskE(obj)
        if obj.timmer_in_use==0
            % initialize a timmer
                obj.timmer_in_use=1;
                obj.current_step=0;
        end
        pos_ref=[5;5;5*(30-obj.current_step)/30];
       

        if all(obj.pos>=pos_ref-obj.tolerance) ...
                && all(obj.pos<=pos_ref+obj.tolerance)...
                && obj.time-obj.last_time>=2
               obj.last_time=obj.time;
               obj.current_step=obj.current_step+1;
         
        end
        if obj.current_step==30
            obj.flags(4)=1;
            obj.timmer_in_use=0;
            obj.current_step=0;
            pos_ref=[5;5;0];
            disp("All tasks are completed");

        end
        end
        
        function input=error2inputs(obj,e,T)
            Ixx=obj.I(1,1);
            Iyy=obj.I(2,2);
            Izz=obj.I(3,3);
%             inputs=zeros(4,1);
%             inputs(1)=T/4-Izz*e(3)/4/obj.b-Ixx*e(1)/2/obj.L/obj.k;
%             inputs(2)=T/4+Izz*e(3)/4/obj.b-Iyy*e(2)/2/obj.L/obj.k;
%             inputs(3)=T/4-Izz*e(3)/4/obj.b+Ixx*e(1)/2/obj.L/obj.k;
%             inputs(4)=T/4+Izz*e(3)/4/obj.b+Iyy*e(2)/2/obj.L/obj.k;
%             
            

            input(1)=T/4-(2*obj.b*e(1)*Ixx+e(3)*Izz*obj.k*obj.L)/(4*obj.b*obj.k*obj.L);
            input(2)= T/4+(e(3)*Izz)/(4*obj.b)-(e(2)*Iyy)/(2*obj.k*obj.L);
            input(3)=T/4-(-2*obj.b*e(1)*Ixx+e(3)*Izz*obj.k*obj.L)/(4*obj.b*obj.k*obj.L);
            input(4)=T/4+(e(3)*Izz)/(4*obj.b)+(e(2)*Iyy)/(2*obj.k*obj.L);
        
        end
        

        function update(obj)
            %update simulation time
            obj.time = obj.time + obj.time_interval;
            obj.pos
            %% get input
            
            num_gamma=obj.m*obj.g/4/obj.k;
            inputs=controller(obj,obj.pos);

            kinematics(obj,inputs)
            
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
            obj.T_B=obj.R*thrust(obj,inputs,k);
            Fd=-kd*xdot;
            a=gravity+1/m*obj.T_B+Fd;
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
            obj.a=acceleration(obj,inputs,obj.theta,obj.xdot,obj.m,obj.g,obj.k,obj.kd);
            omegadot=angular_acceleration(obj,inputs,obj.omega,obj.I,obj.L,obj.b,obj.k);       
            obj.omega=obj.omega+dt*omegadot;
            obj.thetadot=omega2thetadot(obj,obj.omega,obj.theta);
            obj.theta=obj.theta+dt*obj.thetadot;
            obj.xdot=obj.xdot+dt*obj.a;
            obj.pos=obj.pos+dt*obj.xdot;
            
            %record
            obj.posRecord = [obj.posRecord , obj.pos];            
            obj.orienRecord = [obj.orienRecord , obj.theta];
            
        end
    end
end
