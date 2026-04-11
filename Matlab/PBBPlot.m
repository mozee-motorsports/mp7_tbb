%% Live UART logger + live plot (Final, Raw0, Raw1, Pct) with rolling last N points
% Expects lines like:
%   10570 | Final=0 (Raw0=1179 Raw1=1126) Pct=3
%
% Plots the last rollingN samples live, stores everything to memory,
% and saves to MAT/CSV when you close the figure.

clear; clc;

%% -------- USER SETTINGS --------
port = "COM4";                         % Your COM port
baud = 115200;                         % Your baud rate
rollingN = 300;                        % Number of live points shown
saveBaseName = "uart_log_final_raw_pct"; % Base name for output files
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
msgNumAll = [];
finalAll  = [];
raw0All   = [];
raw1All   = [];
pctAll    = [];
tAll      = [];

% Rolling buffers
msg_roll   = nan(rollingN, 1);
raw0_roll  = nan(rollingN, 1);
raw1_roll  = nan(rollingN, 1);
pct_roll   = nan(rollingN, 1);

% Regex for:
%   10570 | Final=0 (Raw0=1179 Raw1=1126) Pct=3
numInt = '([+-]?\d+)';
pat = ['^\s*' ... 
       numInt ...                           % message number
       '\s*\|\s*Final=' numInt ...         % Final
       '\s*\(Raw0=' numInt ...             % Raw0
       '\s+Raw1=' numInt '\)\s*' ...       % Raw1
       'Pct=' numInt '\s*$'];              % Pct

% Figure
fig = figure("Name", "Live UART Final/Raw/Pct", "NumberTitle", "off");
ax = axes(fig);
hold(ax, "on");
grid(ax, "on");
xlabel(ax, "CAN message number");
ylabel(ax, "Value");
title(ax, "Live plot: Raw0, Raw1, and Pct");

hRaw0 = plot(ax, nan, nan, "LineWidth", 1.5);
hRaw1 = plot(ax, nan, nan, "LineWidth", 1.5);
hPct  = plot(ax, nan, nan, "LineWidth", 1.5);

legend(ax, ["Raw0", "Raw1", "Pct"], "Location", "best");

% Stop flag on close
setappdata(fig, 'stop', false);
fig.CloseRequestFcn = @(~,~) setappdata(fig, 'stop', true);

k = 0;
startTic = tic;

disp("Logging. Close the figure window to stop and save.");
disp("Saving files to folder: " + string(outputDir));

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

    msg   = str2double(tok{1});
    final = str2double(tok{2});
    raw0  = str2double(tok{3});
    raw1  = str2double(tok{4});
    pct   = str2double(tok{5});

    % Store full history
    k = k + 1;
    msgNumAll(k,1) = msg;
    finalAll(k,1)  = final;
    raw0All(k,1)   = raw0;
    raw1All(k,1)   = raw1;
    pctAll(k,1)    = pct;
    tAll(k,1)      = toc(startTic);

    % Print parsed values if enabled
    if printParsed && mod(k, printEvery) == 0
        fprintf("PARSED %d | Final=%d Raw0=%d Raw1=%d Pct=%d\n", ...
            msg, final, raw0, raw1, pct);
    end

    % Update rolling buffers
    msg_roll(1:end-1)  = msg_roll(2:end);
    raw0_roll(1:end-1) = raw0_roll(2:end);
    raw1_roll(1:end-1) = raw1_roll(2:end);
    pct_roll(1:end-1)  = pct_roll(2:end);

    msg_roll(end)  = msg;
    raw0_roll(end) = raw0;
    raw1_roll(end) = raw1;
    pct_roll(end)  = pct;

    % Update plot
    valid = ~isnan(msg_roll);
    set(hRaw0, 'XData', msg_roll(valid), 'YData', raw0_roll(valid));
    set(hRaw1, 'XData', msg_roll(valid), 'YData', raw1_roll(valid));
    set(hPct,  'XData', msg_roll(valid), 'YData', pct_roll(valid));

    if any(valid)
        xMin = min(msg_roll(valid));
        xMax = max(msg_roll(valid));
        if xMin == xMax
            xMin = xMin - 1;
            xMax = xMax + 1;
        end
        xlim(ax, [xMin xMax]);
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
    warning("No parsed data points were captured. Check that the UART line exactly matches: 10570 | Final=0 (Raw0=1179 Raw1=1126) Pct=3");
end

% Build output table
T = table(msgNumAll, finalAll, raw0All, raw1All, pctAll, tAll, ...
    'VariableNames', {'msg_num', 'final', 'raw0', 'raw1', 'pct', 'pc_time_s'});

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