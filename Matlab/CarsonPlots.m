%% Plot UART log CSV (R_s and H_s)
% Assumes your CSV has columns like:
% msg_num, pct, R_s, H_s, pc_time_s

clear; clc; close all;

% Option A: pick the file with a dialog
[file, path] = uigetfile("*.csv", "Select your UART log CSV");
if isequal(file,0)
    disp("No file selected.");
    return;
end
csvPath = fullfile(path, file);

% Read CSV into a table
T = readtable(csvPath);

% ---- Choose X axis ----
% If you want "time in CAN messages", use msg_num.
% If you want PC time, use pc_time_s instead.
useMsgNumAsX = false;

if useMsgNumAsX
    x = T.msg_num;
    xLabel = "CAN message number (msg\_num)";
else
    x = T.pc_time_s;
    xLabel = "PC time (s)";
end

% Y data
R = T.R_s;
H = T.H_s;

% Plot
figure("Name","UART Log Plot (R_s, H_s)","NumberTitle","off");
plot(x, R, "LineWidth", 1.5); hold on;
plot(x, H, "LineWidth", 1.5);
grid on;

xlabel(xLabel);
ylabel("ADC taps");
title("R(s) and H(s) from UART log");
legend("R(s)", "H(s)", "Location", "best");

