data = readtable("C:\Users\brynp\Downloads\DAQ_DECODED_0010.csv");
data = renamevars(data, ["Time_uS"], ["micros"])


% 1. Sort by micros to ensure chronological order
data = sortrows(data, 'micros');

% 2. Normalize time so the plot definitely starts at 0
% This subtracts the smallest timestamp from all timestamps
data.micros = data.micros - min(data.micros);

% Filter tables for each sensor
schData  = data(strcmp(data.ID, 'S'), :);
adxlData = data(strcmp(data.ID, 'A'), :);
bnoAccel = data(strcmp(data.ID, 'B') & ~isnan(data.BNO_m_s2_X), :);
bnoGyro  = data(strcmp(data.ID, 'B') & ~isnan(data.BNO_Deg_s_X), :);
bnoMag   = data(strcmp(data.ID, 'B') & ~isnan(data.BNO_Mag_X), :);
%%
figure;

% --- X-Axis Plot ---
subplot(3, 1, 1);
hold on;
plot(adxlData.micros, -1 * adxlData.ADXL_m_s2_X, 'DisplayName', 'ADXL X');
plot(schData.micros, schData.SCH_m_s2_X, 'DisplayName', 'SCH X');
plot(bnoAccel.micros, bnoAccel.BNO_m_s2_X, 'DisplayName', 'BNO X');
title('X-Axis Acceleration'); ylabel('m/s^2'); legend; grid on;
hold off;

% --- Y-Axis Plot ---
subplot(3, 1, 2);
hold on;
plot(adxlData.micros, data.ADXL_m_s2_Y(strcmp(data.ID, 'A'))  , 'DisplayName', 'ADXL Y'); % Alternate filtering syntax
plot(schData.micros, schData.SCH_m_s2_Y, 'DisplayName', 'SCH Y');
plot(bnoAccel.micros, bnoAccel.BNO_m_s2_Y, 'DisplayName', 'BNO Y');
title('Y-Axis Acceleration'); ylabel('m/s^2'); legend; grid on;
hold off;

% --- Z-Axis Plot ---
subplot(3, 1, 3);
hold on;
plot(adxlData.micros, adxlData.ADXL_m_s2_Z  , 'DisplayName', 'ADXL Z');
plot(schData.micros, schData.SCH_m_s2_Z, 'DisplayName', 'SCH Z');
plot(bnoAccel.micros, bnoAccel.BNO_m_s2_Z, 'DisplayName', 'BNO Z');
title('Z-Axis Acceleration'); ylabel('m/s^2'); xlabel('Time (micros)'); legend; grid on;
hold off;

sgtitle('IMU Acceleration Comparison (Time Normalized)');

%%
figure;

% --- X-Axis Plot ---
subplot(3, 1, 1);
hold on;
plot(schData.micros, schData.SCH_Deg_s_X, 'DisplayName', 'SCH X');
plot(bnoGyro.micros, bnoGyro.BNO_Deg_s_X, 'DisplayName', 'BNO X');
title('X-Axis Acceleration'); ylabel('m/s^2'); legend; grid on;
hold off;

% --- Y-Axis Plot ---
subplot(3, 1, 2);
hold on;
plot(schData.micros, schData.SCH_Deg_s_Y, 'DisplayName', 'SCH Y');
plot(bnoGyro.micros, bnoGyro.BNO_Deg_s_Y, 'DisplayName', 'BNO Y');
title('Y-Axis Acceleration'); ylabel('m/s^2'); legend; grid on;
hold off;

% --- Z-Axis Plot ---
subplot(3, 1, 3);
hold on;
%plot(schData.micros, schData.SCH_Deg_s_Z, 'DisplayName', 'SCH Z');
plot(bnoData.micros, rmmissing(bnoData.BNO_Deg_s_Z), 'DisplayName', 'BNO Z');
title('Z-Axis Acceleration'); ylabel('m/s^2'); xlabel('Time (micros)'); legend; grid on;
hold off;

sgtitle('IMU Acceleration Comparison (Time Normalized)');

%%
plot(bnoGyro.micros, bnoGyro.BNO_Deg_s_Z, 'DisplayName', 'BNO Z');
