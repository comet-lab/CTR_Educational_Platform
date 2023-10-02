classdef Drive < handle
    % MOVE class to move the robot

    properties
        % The COM Port here is the port that your computer assigns to the
        % Octopus board ("USB serial device")
        COMPort = "COM13";
        ser;

        % Initialize the home and current positions as Pose objects
        homePose = Pose(0,0,0,0,0,0)
        currPose = Pose(0,0,0,0,0,0)
    end

    methods
        % Constructor takes in a pose object and configures the device to
        % be at that pose
        function self = Drive(currPose)            
            % Set the pose object as the current position.
            self.currPose = currPose;

            % Open the Serial port
            self.ser = serialport(self.COMPort, 250000);
            fopen(self.ser);

            % Set the current pose on the device
            self.set_current_pose(currPose)
        end
        
        % sets the home position on the robot to be the given pose
        function set_home_as_pose(self, pose)
            command = 'G92 ' + pose.get_gcode_for_pose() + '\n';
            self.send_command(command)
        end

        % sets the home position on the robot to be the current pose
        function set_home_as_current(self)
            command = 'G92 ' + self.homePose.get_gcode_for_pose() + '\n';
            self.send_command(command)
        end

        % Sets the current pose to be correct by resetting the home pose on
        % the device 
        function set_current_pose(self, currPose)
            % set home to be 0
            self.set_home_as_pose(self.homePose);

            % set home to be inverse of currPose
            invCurrPose = Pose(-currPose.lin1, -currPose.lin2, -currPose.lin3, -currPose.rot1, -currPose.rot2, -currPose.rot3);
            self.set_home_as_pose(invCurrPose);
        end

        % Travels for the given distance; sets current pose to match
        function travel_for(self, lin1, lin2, lin3, rot1, rot2, rot3)
            % We add the new pose to the current pose
            self.currPose = Pose(self.currPose.lin1 + lin1, self.currPose.lin2 + lin2, self.currPose.lin3 + lin3, self.currPose.rot1 + rot1, self.currPose.rot2 + rot2, self.currPose.rot3 + rot3);
            
            % Then send the device to the new position
            command = 'G0 ' + self.currPose.get_gcode_for_pose() + '\n';
            self.send_command(command)
        end

        % Travels to an absolute distance based on the last defined home
        % position; sets current pose to match
        function travel_to(self, lin1, lin2, lin3, rot1, rot2, rot3)
            % We set the position to be the new position
            self.currPose = Pose(lin1, lin2, lin3, rot1, rot2, rot3);
            
            % Then send the device to that new position
            command = 'G0 ' + self.currPose.get_gcode_for_pose() + '\n';
            self.send_command(command)
        end

        % Wrapper function for sending commands through the serial
        % interface
        function send_command(self, command)
            fprintf(self.ser, command);
        end
    end
end