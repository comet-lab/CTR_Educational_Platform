classdef Robot < handle

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
        function self = Robot(tubes)
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
            % your code goes here
            %
            %
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
            % your code goes here
            %
            %
        end

        % Take in all robot dependent parameters (lls, phi, kappa) and
        % compelte the robot independent constant curvature kinamtatics
        % Returns a 4x4 transformation matrix from base frame to end
        % effector
        function T = calculate_transform(self, s, phi, K)
            % your code goes here
            %
            %
        end
    end
end

