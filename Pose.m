classdef Pose < handle

    properties
        lin1 = 0
        lin2 = 0
        lin3 = 0
        rot1 = 0
        rot2 = 0
        rot3 = 0
    end
    
    methods
        function self = Pose(lin1, lin2, lin3, rot1, rot2, rot3)
            self.lin1 = lin1;
            self.lin2 = lin2;
        	self.lin3 = lin3;
            self.rot1 = rot1;
            self.rot2 = rot2;
            self.rot3 = rot3;
        end
        
        function command = get_gcode_for_pose(self)
            command = "X" + string(self.lin1 * 16) + " Y" + string(self.lin2 * 16) + " Z" + string(self.lin3 * 16) + " A" + string(self.rot1) + " B" + string(self.rot2) + " C" + string(self.rot3);
        end
        
        function command = get_string_for_pose(self)
            command = "(" + string(self.lin1) + ", " + string(self.lin2) + ", " + string(self.lin3) + ", " + string(self.rot1) + ", " + string(self.rot2) + ", " + string(self.rot3) + ")";
        end
        
        function pose_list = get_pose(self)
            pose_list = [self.lin1, self.lin2, self.lin3, self.rot1, self.rot2, self.rot3];
        end
    end
end

