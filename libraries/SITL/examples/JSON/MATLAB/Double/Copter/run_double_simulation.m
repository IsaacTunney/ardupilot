clc
clear all
clearvars
close all
addpath(genpath('../'))

%% Params
init_yaw_leader = 87.5;
init_yaw_follow = 0;

port_leader = 9012;
port_follow = 9002;

%% Call Simulation
fnct_SIM_follow_customModel(port_leader, init_yaw_leader, port_follow, init_yaw_follow)

