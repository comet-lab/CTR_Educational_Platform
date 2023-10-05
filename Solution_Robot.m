classdef Solution_Robot < handle

    properties
        % Save vector of tubes and size of vector
        tubes = []
        num_tubes = 0

        % Save link lengths, phi values, and kappa values vectors (1 x num
        % links)
        lls = []
        phi = []
        kappa = []
    end

    methods
        % Constructor. This creates an instance of the robot class 
        % Must pass in a vector of tubes
        function self = Solution_Robot(tubes)
            self.tubes = tubes;
            self.num_tubes = size(tubes, 2);
        end

        % Here we calculate the kinematics of a full CTR
        % Pass in the raw joint variables
        % Return transformation matrix that goes from home frame to end
        % effector frame
        % See functions below for each step
        function T = fkin(self, q_var)  
            % First we get the rho and theta avlues from q_var
            rho = get_rho_values(self, q_var);
            theta = get_theta(self, q_var);

            % Next, we use rho to get the link lengths
            self.lls = get_links(self, rho);

            % Now we calculate the phi and kappa values
            [self.phi,self.kappa] = calculate_phi_and_kappa(self, theta);

            % Finally we calculate the base to end effector transform
            T = calculate_transform(self, self.lls, self.phi, self.kappa);
        end

        % Get rho values from joint positions
        % Return rho (1 x i vector, i is num tubes)
        function rho = get_rho_values(self, q_var)
            % Here q_var = [lin1, lin2, rot1, rot2] or
            % q _var = [lin1, lin2, lin3, rot1, rot2, rot3]

            % First define rho values in terms of joint variables. We
            % assume that T1 moves with q_var(1). Since we want rho1 to be 0,
            % we have to subtract q_var(1) from other variables. 

            if(self.num_tubes == 2)
                rho = [0, q_var(2) - q_var(1)]*10^-3;
            else
                rho = [0, q_var(2) - q_var(1), q_var(3) - q_var(1)]*10^-3;
            end

        end

        % Function to find the link lengths, in order
        % Returns link lengths (1 x j vector, where j is num links)
        function s = get_links(self, rho)

            if(self.num_tubes == 2)
                d = [self.tubes(1).d, self.tubes(2).d];
                T = [rho(1), rho(2), rho(1)+d(1), rho(2)+d(2)]; 

                Ts = sort(T);
                s = [Ts(2)-Ts(1), Ts(3)-Ts(2), Ts(4)-Ts(3)];
            else
                d = [self.tubes(1).d, self.tubes(2).d, self.tubes(3).d];
                T = [rho(1), rho(2), rho(3), rho(1)+d(1), rho(2)+d(2), rho(3)+d(3)];

                Ts = sort(T);

                s = [Ts(2)-Ts(1), Ts(3)-Ts(2), Ts(4)-Ts(3), Ts(5)-Ts(4), Ts(6)-Ts(5)];
            end
        end

        % Function to get theta values
        % Returns theta (1 x j vector where j is num links)
        function theta = get_theta(self, q_var)
            % We know thete from the joint variables
                
            if (self.num_tubes == 2)
                theta = [q_var(3), q_var(4)];
            else
                theta = [q_var(4), q_var(5), q_var(6)];
            end
        end


        % Function to calcualte phi values for two or three tube
        % configurations
        % Should return phi (1 x j vector, where j is num links)
        % and K (1 x j vector)
        function [phi,K] = calculate_phi_and_kappa(self, theta)
            if(self.num_tubes == 2)
                x1n = self.tubes(1).E*self.tubes(1).I*self.tubes(1).k*cos(theta(1));
                x1 = x1n/(self.tubes(1).E*self.tubes(1).I + self.tubes(2).E*self.tubes(2).I);
        
                x2n = self.tubes(1).E*self.tubes(1).I*self.tubes(1).k*cos(theta(1)) + self.tubes(2).E*self.tubes(2).I*self.tubes(2).k*cos(theta(2));
                x2 = x2n/(self.tubes(1).E*self.tubes(1).I + self.tubes(2).E*self.tubes(2).I);
        
                x3n = self.tubes(2).E*self.tubes(2).I*self.tubes(2).k*cos(theta(2));
                x3 = x3n/(self.tubes(2).E*self.tubes(2).I);
        
        
                y1n = self.tubes(1).E*self.tubes(1).I*self.tubes(1).k*sin(theta(1));
                y1 = y1n/(self.tubes(1).E*self.tubes(1).I + self.tubes(2).E*self.tubes(2).I);
        
                y2n = self.tubes(1).E*self.tubes(1).I*self.tubes(1).k*sin(theta(1)) + self.tubes(2).E*self.tubes(2).I*self.tubes(2).k*sin(theta(2));
                y2 = y2n/(self.tubes(1).E*self.tubes(1).I + self.tubes(2).E*self.tubes(2).I);
        
                y3n = self.tubes(2).E*self.tubes(2).I*self.tubes(2).k*sin(theta(2));
                y3 = y3n/(self.tubes(2).E*self.tubes(2).I);
        
                phi1 = atan2(y1, x1);
                phi2 = atan2(y2, x2);
                phi3 = atan2(y3, x3);
        
                phi = [phi1, phi2, phi3];

                K1 = sqrt(x1*x1 + y1*y1);
                K2 = sqrt(x2*x2 + y2*y2);
                K3 = sqrt(x3*x3 + y3*y3);
        
                K = [K1, K2, K3];
            else
                x1n = self.tubes(1).E*self.tubes(1).I*self.tubes(1).k*cos(theta(1));
                x1 = x1n/(self.tubes(1).E*self.tubes(1).I + self.tubes(2).E*self.tubes(2).I + self.tubes(3).E*self.tubes(3).I);
        
                x2n = self.tubes(1).E*self.tubes(1).I*self.tubes(1).k*cos(theta(1)) + self.tubes(2).E*self.tubes(2).I*self.tubes(2).k*cos(theta(2));
                x2 = x2n/(self.tubes(1).E*self.tubes(1).I + self.tubes(2).E*self.tubes(2).I + self.tubes(3).E*self.tubes(3).I);
        
                x3n = self.tubes(2).E*self.tubes(2).I*self.tubes(2).k*cos(theta(2));
                x3 = x3n/(self.tubes(2).E*self.tubes(2).I + self.tubes(3).E*self.tubes(3).I);
    
                x4n = self.tubes(2).E*self.tubes(2).I*self.tubes(2).k*cos(theta(2)) + self.tubes(3).E*self.tubes(3).I*self.tubes(3).k*cos(theta(3));
                x4 = x4n/(self.tubes(2).E*self.tubes(2).I + self.tubes(3).E*self.tubes(3).I);
    
                x5n = self.tubes(3).E*self.tubes(3).I*self.tubes(3).k*cos(theta(3));
                x5 = x5n/(self.tubes(3).E*self.tubes(3).I);
        
        
                y1n = self.tubes(1).E*self.tubes(1).I*self.tubes(1).k*sin(theta(1));
                y1 = y1n/(self.tubes(1).E*self.tubes(1).I + self.tubes(2).E*self.tubes(2).I + self.tubes(3).E*self.tubes(3).I);
        
                y2n = self.tubes(1).E*self.tubes(1).I*self.tubes(1).k*sin(theta(1)) + self.tubes(2).E*self.tubes(2).I*self.tubes(2).k*sin(theta(2));
                y2 = y2n/(self.tubes(1).E*self.tubes(1).I + self.tubes(2).E*self.tubes(2).I + self.tubes(3).E*self.tubes(3).I);
        
                y3n = self.tubes(2).E*self.tubes(2).I*self.tubes(2).k*sin(theta(2));
                y3 = y3n/(self.tubes(2).E*self.tubes(2).I + self.tubes(3).E*self.tubes(3).I);
    
                y4n = self.tubes(2).E*self.tubes(2).I*self.tubes(2).k*sin(theta(2)) + self.tubes(3).E*self.tubes(3).I*self.tubes(3).k*sin(theta(3));
                y4 = y4n/(self.tubes(2).E*self.tubes(2).I + self.tubes(3).E*self.tubes(3).I);
    
                y5n = self.tubes(3).E*self.tubes(3).I*self.tubes(3).k*sin(theta(3));
                y5 = y5n/(self.tubes(3).E*self.tubes(3).I);
        
                phi1 = atan2(y1, x1);
                phi2 = atan2(y2, x2);
                phi3 = atan2(y3, x3);
                phi4 = atan2(y4, x4);
                phi5 = atan2(y5, x5);
        
                phi = [phi1, phi2, phi3, phi4, phi5];

                K1 = sqrt(x1*x1 + y1*y1);
                K2 = sqrt(x2*x2 + y2*y2);
                K3 = sqrt(x3*x3 + y3*y3);
                K4 = sqrt(x4*x4 + y4*y4);
                K5 = sqrt(x5*x5 + y5*y5);
        
                K = [K1, K2, K3, K4, K5];
            end
        end

        % Take in all robot dependent parameters (lls, phi, kappa) and
        % compelte the robot independent constant curvature kinamtatics
        % Returns a 4x4 transformation matrix from base frame to end
        % effector
        function T = calculate_transform(self, s, phi, K)
            T=[];

            if(self.num_tubes ==2)
                for i =1:3    
                    tt = [[(cos(phi(i))*cos(phi(i))*(cos(K(i)*s(i))-1)) + 1, sin(phi(i))*cos(phi(i))*(cos(K(i)*s(i))-1), cos(phi(i))*sin(K(i)*s(i)), cos(phi(i))*(1-cos(K(i)*s(i)))/K(i)];
                        [sin(phi(i))*cos(phi(i))*(cos(K(i)*s(i)) -1), + cos(phi(i))*cos(phi(i))*(1 - cos(K(i)*s(i))) + cos(K(i)*s(i)), sin(phi(i))*sin(K(i)*s(i)), sin(phi(i))*(1-cos(K(i)*s(i)))/K(i)];
                        [-cos(phi(i))*sin(K(i)*s(i)), -sin(phi(i))*sin(K(i)*s(i)), cos(K(i)*s(i)), sin(K(i)*s(i))/K(i)];
                        [0, 0, 0, 1]];
                   
                    
                    T(:,:,i) = tt;
                end
            else
                for i =1:5
                    tt = [[cos(phi(i))*cos(K(i)*s(i)), -sin(phi(i)), cos(phi(i))*sin(K(i)*s(i)), cos(phi(i))*(1-cos(K(i)*s(i)))/K(i)];
                        [sin(phi(i))*cos(K(i)*s(i)), +cos(phi(i)), sin(phi(i))*sin(K(i)*s(i)), sin(phi(i))*(1-cos(K(i)*s(i)))/K(i)];
                        [-sin(K(i)*s(i)), 0, cos(K(i)*s(i)), sin(K(i)*s(i))/K(i)];
                        [0, 0, 0, 1]];
                    
                    T(:,:,i) = tt;
                end
            end
        end
    end
end

