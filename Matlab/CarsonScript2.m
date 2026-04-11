%% Live UART logger + live plot (R(s), H(s)) with rolling last N points
% Expects lines like:
%   2512 | R(s): 1517.000000, H(s): 3401.000000
% Plots the last rollingN samples live.
% Optionally stores full history to memory and saves MAT/CSV when you close the figure.

clear; clc;

%% -------- USER SETTINGS --------
port = "COM4";                         % Your COM port
baud = 115200;                         % Your baud rate
rollingN = 300;                        % Number of live points shown

enableDataLogging = false;              % true = store full history and save MAT/CSV, false = live plot only

saveBaseName = "uart_log_rs_hs";       % Base name for output files
outputDir = "C:\mozee\mp7_tbb\Matlab"; % Folder to save into
useTimestamp = true;                   % Append timestamp to avoid overwrite

printRawUart = true;                   % Print every UART line
printParsed = false;                   % Print parsed values
printEvery = 1;                        % If printParsed=true, print every N parsed lines
%% -------------------------------

% Create serial port
s = serialport(port, baud);
configureTerminator(s, "CR/LF");   % Matches \r\n
s.Timeout = 1;
flush(s);

% Full data storage
if enableDataLogging
    msgNumAll = [];
    R_all     = [];
    H_all     = [];
    tAll      = [];
else
    msgNumAll = [];
    R_all     = [];
    H_all     = [];
    tAll      = [];
end

% Rolling buffers
msg_roll = nan(rollingN, 1);
R_roll   = nan(rollingN, 1);
H_roll   = nan(rollingN, 1);

% Regex for:
%   2512 | R(s): 1517.000000, H(s): 3401.000000
% and also allows extra text after H(s), for example:
%   2512 | R(s): 1517.000000, H(s): 3401.000000, PID: 12.5
num = '([+-]?\d+(?:\.\d*)?(?:[eE][+-]?\d+)?)';
pat = ['^\s*(\d+)\s*\|\s*R\(s\):\s*' num '\s*,\s*H\(s\):\s*' num '.*$'];

% Figure
fig = figure("Name", "Live UART R(s)/H(s)", "NumberTitle", "off");
ax = axes(fig);
hold(ax, "on");
grid(ax, "on");
xlabel(ax, "CAN message number");
ylabel(ax, "Percent open throttle");
title(ax, "Live plot: R(s) and H(s)");

hR = plot(ax, nan, nan, "LineWidth", 1.5);
hH = plot(ax, nan, nan, "LineWidth", 1.5);
legend(ax, ["R(s)", "H(s)"], "Location", "best");

% Stop flag on close
setappdata(fig, 'stop', false);
fig.CloseRequestFcn = @(~,~) setappdata(fig, 'stop', true);

k = 0;
startTic = tic;

disp("Logging. Close the figure window to stop.");
if enableDataLogging
    disp("Data logging is ON.");
    disp("Saving files to folder: " + string(outputDir));
else
    disp("Data logging is OFF. No MAT/CSV files will be saved.");
end

while isvalid(fig) && ~getappdata(fig, 'stop')
    if s.NumBytesAvailable == 0
        pause(0.01);
        continue;
    end

    raw = readline(s);

    % Convert safely to string
    if isstring(raw) || ischar(raw)
        line = string(raw);
    else
        line = string(char(raw(:).'));
    end
    line = strtrim(line);

    % Print raw UART line
    if printRawUart
        fprintf("%s\n", line);
    end

    % Parse line
    tok = regexp(char(line), pat, 'tokens', 'once');
    if isempty(tok)
        continue;
    end

    msg = str2double(tok{1});
    Rv  = str2double(tok{2});
    Hv  = str2double(tok{3});

    % Count parsed points
    k = k + 1;

    % Store full history only if enabled
    if enableDataLogging
        msgNumAll(k,1) = msg;
        R_all(k,1)     = Rv;
        H_all(k,1)     = Hv;
        tAll(k,1)      = toc(startTic);
    end

    % Print parsed values if enabled
    if printParsed && mod(k, printEvery) == 0
        fprintf("PARSED %d | R(s): %.6f, H(s): %.6f\n", msg, Rv, Hv);
    end

    % Update rolling buffers
    msg_roll(1:end-1) = msg_roll(2:end);
    R_roll(1:end-1)   = R_roll(2:end);
    H_roll(1:end-1)   = H_roll(2:end);

    msg_roll(end) = msg;
    R_roll(end)   = Rv;
    H_roll(end)   = Hv;

    % Update plot
    valid = ~isnan(msg_roll);
    set(hR, 'XData', msg_roll(valid), 'YData', R_roll(valid));
    set(hH, 'XData', msg_roll(valid), 'YData', H_roll(valid));

    if any(valid)
        xMin = min(msg_roll(valid));
        xMax = max(msg_roll(valid));
        if xMin == xMax
            xMin = xMin - 1;
            xMax = xMax + 1;
        end
        xlim(ax, [xMin xMax]);
        ylim(ax, [0 20]);
    end

    drawnow limitrate;
end

% Close figure cleanly
if isvalid(fig)
    delete(fig);
end

% Clean up serial
try
    flush(s);
    clear s;
catch
end

% Warn if nothing parsed
if k == 0
    warning("No parsed data points were captured. Check that the UART line exactly matches: 2512 | R(s): 1517.000000, H(s): 3401.000000");
end

% Save files only if data logging is enabled
if enableDataLogging
    % Build output table
    T = table(msgNumAll, R_all, H_all, tAll, ...
        'VariableNames', {'msg_num', 'R_s', 'H_s', 'pc_time_s'});

    % Make output folder if needed
    if ~isfolder(outputDir)
        mkdir(outputDir);
    end

    % Build filename
    if useTimestamp
        stamp = string(datetime("now", "Format", "yyyyMMdd_HHmmss"));
        matFile = fullfile(outputDir, saveBaseName + "_" + stamp + ".mat");
        csvFile = fullfile(outputDir, saveBaseName + "_" + stamp + ".csv");
    else
        matFile = fullfile(outputDir, saveBaseName + ".mat");
        csvFile = fullfile(outputDir, saveBaseName + ".csv");
    end

    % Save files
    save(matFile, "T");
    writetable(T, csvFile);

    disp("Saved MAT file: " + matFile);
    disp("Saved CSV file: " + csvFile);
else
    disp("Run complete. Data logging was disabled, so no files were saved.");
end