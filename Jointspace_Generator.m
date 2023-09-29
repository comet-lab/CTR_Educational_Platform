classdef Jointspace_Generator < handle

    properties
        cart1 = Cart(0, 30, -45, 45);
        cart2 = Cart(0, 30, -90, 90);
        cart3 = Cart(0, 0, 0, 0);
        
        % Lists or the positions in different formats
        positions_list = []
        positions_list_string = []
        positions_gcode_string = ""
        positions_gcode_list = []
    end
    
    methods
        function self = Jointspace_Generator()
        end
        
        % Creates and returns array of random positions as Pose objects in 
        % joint space format
        function positions_list = get_new_random_positions(self,num_positions)
            
            positions_list = [Pose(0,0,0,0,0,0)];
            rng('shuffle'); % This makes positions random but will return the same random position each time
            randi([-10 10],1,1000);
            for i = 1:num_positions
                lin1 = randi(self.cart1.get_lin_range(), 1, 1);
                lin2 = randi(self.cart2.get_lin_range(), 1, 1);
                lin3 = randi(self.cart3.get_lin_range(), 1, 1);
%                 lin3 = 0; % Use this for two tube tests
                rot1 = randi(self.cart1.get_rot_range(), 1, 1);
                rot2 = randi(self.cart2.get_rot_range(), 1, 1);
                rot3 = randi(self.cart3.get_rot_range(), 1, 1);

                new_pose = Pose(lin1, (lin1+lin2), (lin1+lin2+lin3), rot1, rot2, rot3);
                positions_list(i+1,1) = new_pose;

                self.positions_list = positions_list;
                self.positions_list_string = self.positions_list_to_string();
                self.positions_gcode_string = self.positions_list_to_gcode_string();
                self.positions_gcode_list = self.positions_list_to_gcode_list();
            end
        end

        % Returns a string of joint positions; not used for any code --
        % diagnostic only
        function string = positions_list_to_string(self)
            string = "";
            for i = 1:size(self.positions_list, 1)
                pose = self.positions_list(i,1);
                command = pose.get_string_for_pose();
                string = string + command + "\n";
            end
        end
        
       % Returns string of gcode for positions; not used for any code --
       % diagnostic only
       function string = positions_list_to_gcode_string(self)
            string = "";
            for i = 1:size(self.positions_list, 1)
                pose = self.positions_list(i,1);
                command = pose.get_gcode_for_pose();
                string = string + command + "\n";
            end
       end
        
       % Returns an array of gcode strings that can be sent to the robot
       % as commands.
       function gcode_list = positions_list_to_gcode_list(self)
            gcode_list = [""];
            for i = 1:size(self.positions_list, 1)
                pose = self.positions_list(i,1);
                command = pose.get_gcode_for_pose();
                gcode_list(i,1) = command + "\n";
            end
       end
    end
end

