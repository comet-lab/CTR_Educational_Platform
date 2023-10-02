%% Set up
clear; clc;

startPose = Pose(0, 0, 0, 0, 0, 0);
drive_bot = Drive(startPose);

%% Test linear 1
drive_bot.travel_for(10,0,0,0,0,0)

%% Test linear 2
drive_bot.travel_for(0,10,0,0,0,0)

%% Test linear 3
drive_bot.travel_for(0,0,10,0,0,0)

%% Test rotation 1
drive_bot.travel_for(0,0,0,90,0,0)

%% Test rotation 2
drive_bot.travel_for(0,0,0,0,90,0)

%% Test rotation 3
drive_bot.travel_for(0,0,0,0,0,90)