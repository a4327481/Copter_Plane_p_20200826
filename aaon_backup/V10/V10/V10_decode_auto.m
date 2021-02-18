function V10Log = V10_decode_auto(logFile)
% example: V10Log = V10_decode_auto('log_108.bin-643947.mat')
% computer name: LAPTOP-KCGKQN65
% generate date: 18-Jan-2021
% Matlab version: 9.9.0.1467703 (R2020b)
% protocol file: V10_v20210112.txt
% data file: log_108.bin-643947.mat
% logFile: .mat log file
load(logFile);
HD=180/pi;
%% ARP1
V10Log.ARP1.TimeUS = ARP1(:,2);                        %   1. | TimeUS  | TimeUS                        | [U64] | [10:10]   |
V10Log.ARP1.CNT = ARP1(:,3);                           %   2. | CNT     | CNT                           | [U32] | [10:10]   |
V10Log.ARP1.air_temp = ARP1(:,4);                      %   3. | tmp1    | air_temp                      | [F]   | [10:10]   |
V10Log.ARP1.air_diff_press_pa_raw = ARP1(:,5);         %   4. | prs1    | air_diff_press_pa_raw         | [F]   | [10:10]   |
V10Log.ARP1.indicated_airspeed = ARP1(:,6);            %   5. | isp1    | indicated_airspeed            | [F]   | [10:10]   |
V10Log.ARP1.true_airspeed = ARP1(:,7);                 %   6. | tsp1    | true_airspeed                 | [F]   | [10:10]   |
V10Log.ARP1.I2C_AirRetryCount_0_0 = ARP1(:,8);         %   7. | err1    | I2C_AirRetryCount_0_0         | [U32] | [10:10]   |
V10Log.ARP1.I2C_AirRetryCount_0_1 = ARP1(:,9);         %   8. | err2    | I2C_AirRetryCount_0_1         | [U32] | [10:10]   |
V10Log.ARP1.Sum = ARP1(:,10);                          %   9. | Sum     | Sum                           | [U8]  | [10:10]   |
%% ARP2
% V10Log.ARP2.TimeUS = ARP2(:,2);                        %   1. | TimeUS  | TimeUS                        | [U64] | [10:10]   |
% V10Log.ARP2.CNT = ARP2(:,3);                           %   2. | CNT     | CNT                           | [U32] | [10:10]   |
% V10Log.ARP2.air_temp = ARP2(:,4);                      %   3. | tmp2    | air_temp                      | [F]   | [10:10]   |
% V10Log.ARP2.air_diff_press_pa_raw = ARP2(:,5);         %   4. | prs2    | air_diff_press_pa_raw         | [F]   | [10:10]   |
% V10Log.ARP2.indicated_airspeed = ARP2(:,6);            %   5. | isp2    | indicated_airspeed            | [F]   | [10:10]   |
% V10Log.ARP2.true_airspeed = ARP2(:,7);                 %   6. | tsp2    | true_airspeed                 | [F]   | [10:10]   |
% V10Log.ARP2.I2C_AirRetryCount_1_0 = ARP2(:,8);         %   7. | err1    | I2C_AirRetryCount_1_0         | [U32] | [10:10]   |
% V10Log.ARP2.I2C_AirRetryCount_1_1 = ARP2(:,9);         %   8. | err2    | I2C_AirRetryCount_1_1         | [U32] | [10:10]   |
% V10Log.ARP2.Sum = ARP2(:,10);                          %   9. | Sum     | Sum                           | [U8]  | [10:10]   |
%% MAG1
V10Log.MAG1.TimeUS = MAG1(:,2);                        %   1. | TimeUS  | TimeUS                        | [U64] | [10:10]   |
V10Log.MAG1.CNT = MAG1(:,3);                           %   2. | CNT     | CNT                           | [U32] | [10:10]   |
V10Log.MAG1.true_data_x = MAG1(:,4);                   %   3. | Lx_t    | true_data_x                   | [F]   | [10:10]   |
V10Log.MAG1.true_data_y = MAG1(:,5);                   %   4. | Ly_t    | true_data_y                   | [F]   | [10:10]   |
V10Log.MAG1.true_data_z = MAG1(:,6);                   %   5. | Lz_t    | true_data_z                   | [F]   | [10:10]   |
V10Log.MAG1.cali_data_x = MAG1(:,7);                   %   6. | Lx_c    | cali_data_x                   | [F]   | [10:10]   |
V10Log.MAG1.cali_data_y = MAG1(:,8);                   %   7. | Ly_c    | cali_data_y                   | [F]   | [10:10]   |
V10Log.MAG1.cali_data_z = MAG1(:,9);                   %   8. | Lz_c    | cali_data_z                   | [F]   | [10:10]   |
V10Log.MAG1.I2C_MagRetryCount_0_0 = MAG1(:,10);        %   9. | err1    | I2C_MagRetryCount_0_0         | [U32] | [10:10]   |
V10Log.MAG1.I2C_MagRetryCount_0_1 = MAG1(:,11);        %  10. | err2    | I2C_MagRetryCount_0_1         | [U32] | [10:10]   |
V10Log.MAG1.Sum = MAG1(:,12);                          %  11. | Sum     | Sum                           | [U8]  | [10:10]   |
%% MAG2
% V10Log.MAG2.TimeUS = MAG2(:,2);                        %   1. | TimeUS  | TimeUS                        | [U64] | [10:10]   |
% V10Log.MAG2.CNT = MAG2(:,3);                           %   2. | CNT     | CNT                           | [U32] | [10:10]   |
% V10Log.MAG2.true_data_x = MAG2(:,4);                   %   3. | Rx_t    | true_data_x                   | [F]   | [10:10]   |
% V10Log.MAG2.true_data_y = MAG2(:,5);                   %   4. | Ry_t    | true_data_y                   | [F]   | [10:10]   |
% V10Log.MAG2.true_data_z = MAG2(:,6);                   %   5. | Rz_t    | true_data_z                   | [F]   | [10:10]   |
% V10Log.MAG2.cali_data_x = MAG2(:,7);                   %   6. | Rx_c    | cali_data_x                   | [F]   | [10:10]   |
% V10Log.MAG2.cali_data_y = MAG2(:,8);                   %   7. | Ry_c    | cali_data_y                   | [F]   | [10:10]   |
% V10Log.MAG2.cali_data_z = MAG2(:,9);                   %   8. | Rz_c    | cali_data_z                   | [F]   | [10:10]   |
% V10Log.MAG2.I2C_MagRetryCount_1_0 = MAG2(:,10);        %   9. | err1    | I2C_MagRetryCount_1_0         | [U32] | [10:10]   |
% V10Log.MAG2.I2C_MagRetryCount_1_1 = MAG2(:,11);        %  10. | err2    | I2C_MagRetryCount_1_1         | [U32] | [10:10]   |
% V10Log.MAG2.Sum = MAG2(:,12);                          %  11. | Sum     | Sum                           | [U8]  | [10:10]   |
%% BAR1
V10Log.BAR1.TimeUS = BAR1(:,2);                        %   1. | TimeUS  | TimeUS                        | [U64] | [10:10]   |
V10Log.BAR1.CNT = BAR1(:,3);                           %   2. | CNT     | CNT                           | [U32] | [10:10]   |
V10Log.BAR1.ground_pressure = BAR1(:,4);               %   3. | gP      | ground_pressure               | [F]   | [10:10]   |
V10Log.BAR1.ground_temperature = BAR1(:,5);            %   4. | gT      | ground_temperature            | [F]   | [10:10]   |
V10Log.BAR1.pressure = BAR1(:,6);                      %   5. | pres    | pressure                      | [F]   | [10:10]   |
V10Log.BAR1.altitude = BAR1(:,7);                      %   6. | alt     | altitude                      | [F]   | [10:10]   |
V10Log.BAR1.temperature = BAR1(:,8);                   %   7. | temp    | temperature                   | [F]   | [10:10]   |
V10Log.BAR1.Sum = BAR1(:,9);                           %   8. | Sum     | Sum                           | [U8]  | [10:10]   |
%% BAR2
V10Log.BAR2.TimeUS = BAR2(:,2);                        %   1. | TimeUS  | TimeUS                        | [U64] | [10:10]   |
V10Log.BAR2.CNT = BAR2(:,3);                           %   2. | CNT     | CNT                           | [U32] | [10:10]   |
V10Log.BAR2.ground_pressure = BAR2(:,4);               %   3. | gP      | ground_pressure               | [F]   | [10:10]   |
V10Log.BAR2.ground_temperature = BAR2(:,5);            %   4. | gT      | ground_temperature            | [F]   | [10:10]   |
V10Log.BAR2.pressure = BAR2(:,6);                      %   5. | pres    | pressure                      | [F]   | [10:10]   |
V10Log.BAR2.altitude = BAR2(:,7);                      %   6. | alt     | altitude                      | [F]   | [10:10]   |
V10Log.BAR2.temperature = BAR2(:,8);                   %   7. | temp    | temperature                   | [F]   | [10:10]   |
V10Log.BAR2.Sum = BAR2(:,9);                           %   8. | Sum     | Sum                           | [U8]  | [10:10]   |
%% IMU1
V10Log.IMU1.TimeUS = IMU1(:,2);                        %   1. | TimeUS  | TimeUS                        | [U64] | [1:1]     |
V10Log.IMU1.CNT = IMU1(:,3);                           %   2. | CNT     | CNT                           | [U32] | [1:1]     |
V10Log.IMU1.ax = IMU1(:,4);                            %   3. | ax      | ax                            | [F]   | [1:1]     |
V10Log.IMU1.ay = IMU1(:,5);                            %   4. | ay      | ay                            | [F]   | [1:1]     |
V10Log.IMU1.az = IMU1(:,6);                            %   5. | az      | az                            | [F]   | [1:1]     |
V10Log.IMU1.gx = IMU1(:,7);                            %   6. | gx      | gx                            | [F]   | [1:1]     |
V10Log.IMU1.gy = IMU1(:,8);                            %   7. | gy      | gy                            | [F]   | [1:1]     |
V10Log.IMU1.gz = IMU1(:,9);                            %   8. | gz      | gz                            | [F]   | [1:1]     |
V10Log.IMU1.temperature = IMU1(:,10);                  %   9. | temp    | temperature                   | [F]   | [1:1]     |
V10Log.IMU1.temp_pwm = IMU1(:,11);                     %  10. | pwm     | temp_pwm                      | [F]   | [1:1]     |
V10Log.IMU1.Sum = IMU1(:,12);                          %  11. | Sum     | Sum                           | [U8]  | [1:1]     |
%% IMU2
V10Log.IMU2.TimeUS = IMU2(:,2);                        %   1. | TimeUS  | TimeUS                        | [U64] | [1:1]     |
V10Log.IMU2.CNT = IMU2(:,3);                           %   2. | CNT     | CNT                           | [U32] | [1:1]     |
V10Log.IMU2.ax = IMU2(:,4);                            %   3. | ax      | ax                            | [F]   | [1:1]     |
V10Log.IMU2.ay = IMU2(:,5);                            %   4. | ay      | ay                            | [F]   | [1:1]     |
V10Log.IMU2.az = IMU2(:,6);                            %   5. | az      | az                            | [F]   | [1:1]     |
V10Log.IMU2.gx = IMU2(:,7);                            %   6. | gx      | gx                            | [F]   | [1:1]     |
V10Log.IMU2.gy = IMU2(:,8);                            %   7. | gy      | gy                            | [F]   | [1:1]     |
V10Log.IMU2.gz = IMU2(:,9);                            %   8. | gz      | gz                            | [F]   | [1:1]     |
V10Log.IMU2.temperature = IMU2(:,10);                  %   9. | temp    | temperature                   | [F]   | [1:1]     |
V10Log.IMU2.temp_pwm = IMU2(:,11);                     %  10. | pwm     | temp_pwm                      | [F]   | [1:1]     |
V10Log.IMU2.Sum = IMU2(:,12);                          %  11. | Sum     | Sum                           | [U8]  | [1:1]     |
%% IMU3
V10Log.IMU3.TimeUS = IMU3(:,2);                        %   1. | TimeUS  | TimeUS                        | [U64] | [1:1]     |
V10Log.IMU3.CNT = IMU3(:,3);                           %   2. | CNT     | CNT                           | [U32] | [1:1]     |
V10Log.IMU3.ax = IMU3(:,4);                            %   3. | ax      | ax                            | [F]   | [1:1]     |
V10Log.IMU3.ay = IMU3(:,5);                            %   4. | ay      | ay                            | [F]   | [1:1]     |
V10Log.IMU3.az = IMU3(:,6);                            %   5. | az      | az                            | [F]   | [1:1]     |
V10Log.IMU3.gx = IMU3(:,7);                            %   6. | gx      | gx                            | [F]   | [1:1]     |
V10Log.IMU3.gy = IMU3(:,8);                            %   7. | gy      | gy                            | [F]   | [1:1]     |
V10Log.IMU3.gz = IMU3(:,9);                            %   8. | gz      | gz                            | [F]   | [1:1]     |
V10Log.IMU3.temperature = IMU3(:,10);                  %   9. | temp    | temperature                   | [F]   | [1:1]     |
V10Log.IMU3.temp_pwm = IMU3(:,11);                     %  10. | pwm     | temp_pwm                      | [F]   | [1:1]     |
V10Log.IMU3.Sum = IMU3(:,12);                          %  11. | Sum     | Sum                           | [U8]  | [1:1]     |
%% IMU4
V10Log.IMU4.TimeUS = IMU4(:,2);                        %   1. | TimeUS  | TimeUS                        | [U64] | [1:1]     |
V10Log.IMU4.CNT = IMU4(:,3);                           %   2. | CNT     | CNT                           | [U32] | [1:1]     |
V10Log.IMU4.ax = IMU4(:,4);                            %   3. | ax      | ax                            | [F]   | [1:1]     |
V10Log.IMU4.ay = IMU4(:,5);                            %   4. | ay      | ay                            | [F]   | [1:1]     |
V10Log.IMU4.az = IMU4(:,6);                            %   5. | az      | az                            | [F]   | [1:1]     |
V10Log.IMU4.gx = IMU4(:,7);                            %   6. | gx      | gx                            | [F]   | [1:1]     |
V10Log.IMU4.gy = IMU4(:,8);                            %   7. | gy      | gy                            | [F]   | [1:1]     |
V10Log.IMU4.gz = IMU4(:,9);                            %   8. | gz      | gz                            | [F]   | [1:1]     |
V10Log.IMU4.temperature = IMU4(:,10);                  %   9. | temp    | temperature                   | [F]   | [1:1]     |
V10Log.IMU4.temp_pwm = IMU4(:,11);                     %  10. | pwm     | temp_pwm                      | [F]   | [1:1]     |
V10Log.IMU4.Sum = IMU4(:,12);                          %  11. | Sum     | Sum                           | [U8]  | [1:1]     |
%% IMF1
V10Log.IMF1.TimeUS = IMF1(:,2);                        %   1. | TimeUS  | TimeUS                        | [U64] | [1:1]     |
V10Log.IMF1.CNT = IMF1(:,3);                           %   2. | CNT     | CNT                           | [U32] | [1:1]     |
V10Log.IMF1.ax = IMF1(:,4);                            %   3. | ax      | ax                            | [F]   | [1:1]     |
V10Log.IMF1.ay = IMF1(:,5);                            %   4. | ay      | ay                            | [F]   | [1:1]     |
V10Log.IMF1.az = IMF1(:,6);                            %   5. | az      | az                            | [F]   | [1:1]     |
V10Log.IMF1.gx = IMF1(:,7);                            %   6. | gx      | gx                            | [F]   | [1:1]     |
V10Log.IMF1.gy = IMF1(:,8);                            %   7. | gy      | gy                            | [F]   | [1:1]     |
V10Log.IMF1.gz = IMF1(:,9);                            %   8. | gz      | gz                            | [F]   | [1:1]     |
V10Log.IMF1.Sum = IMF1(:,10);                          %   9. | Sum     | Sum                           | [U8]  | [1:1]     |
%% IMF5
V10Log.IMF5.TimeUS = IMF5(:,2);                        %   1. | TimeUS  | TimeUS                        | [U64] | [1:1]     |
V10Log.IMF5.CNT = IMF5(:,3);                           %   2. | CNT     | CNT                           | [U32] | [1:1]     |
V10Log.IMF5.ax = IMF5(:,4);                            %   3. | ax      | ax                            | [F]   | [1:1]     |
V10Log.IMF5.ay = IMF5(:,5);                            %   4. | ay      | ay                            | [F]   | [1:1]     |
V10Log.IMF5.az = IMF5(:,6);                            %   5. | az      | az                            | [F]   | [1:1]     |
V10Log.IMF5.gx = IMF5(:,7);                            %   6. | gx      | gx                            | [F]   | [1:1]     |
V10Log.IMF5.gy = IMF5(:,8);                            %   7. | gy      | gy                            | [F]   | [1:1]     |
V10Log.IMF5.gz = IMF5(:,9);                            %   8. | gz      | gz                            | [F]   | [1:1]     |
V10Log.IMF5.Sum = IMF5(:,10);                          %   9. | Sum     | Sum                           | [U8]  | [1:1]     |
%% IMF6
V10Log.IMF6.TimeUS = IMF6(:,2);                        %   1. | TimeUS  | TimeUS                        | [U64] | [1:1]     |
V10Log.IMF6.CNT = IMF6(:,3);                           %   2. | CNT     | CNT                           | [U32] | [1:1]     |
V10Log.IMF6.ax = IMF6(:,4);                            %   3. | ax      | ax                            | [F]   | [1:1]     |
V10Log.IMF6.ay = IMF6(:,5);                            %   4. | ay      | ay                            | [F]   | [1:1]     |
V10Log.IMF6.az = IMF6(:,6);                            %   5. | az      | az                            | [F]   | [1:1]     |
V10Log.IMF6.gx = IMF6(:,7);                            %   6. | gx      | gx                            | [F]   | [1:1]     |
V10Log.IMF6.gy = IMF6(:,8);                            %   7. | gy      | gy                            | [F]   | [1:1]     |
V10Log.IMF6.gz = IMF6(:,9);                            %   8. | gz      | gz                            | [F]   | [1:1]     |
V10Log.IMF6.Sum = IMF6(:,10);                          %   9. | Sum     | Sum                           | [U8]  | [1:1]     |
%% IMF7
V10Log.IMF7.TimeUS = IMF7(:,2);                        %   1. | TimeUS  | TimeUS                        | [U64] | [1:1]     |
V10Log.IMF7.CNT = IMF7(:,3);                           %   2. | CNT     | CNT                           | [U32] | [1:1]     |
V10Log.IMF7.ax = IMF7(:,4);                            %   3. | ax      | ax                            | [F]   | [1:1]     |
V10Log.IMF7.ay = IMF7(:,5);                            %   4. | ay      | ay                            | [F]   | [1:1]     |
V10Log.IMF7.az = IMF7(:,6);                            %   5. | az      | az                            | [F]   | [1:1]     |
V10Log.IMF7.gx = IMF7(:,7);                            %   6. | gx      | gx                            | [F]   | [1:1]     |
V10Log.IMF7.gy = IMF7(:,8);                            %   7. | gy      | gy                            | [F]   | [1:1]     |
V10Log.IMF7.gz = IMF7(:,9);                            %   8. | gz      | gz                            | [F]   | [1:1]     |
V10Log.IMF7.Sum = IMF7(:,10);                          %   9. | Sum     | Sum                           | [U8]  | [1:1]     |
%% GPS
V10Log.GPS.TimeUS = GPS(:,2);                          %   1. | TimeUS  | TimeUS                        | [U64] | [50:50]   |
V10Log.GPS.CNT = GPS(:,3);                             %   2. | CNT     | CNT                           | [U32] | [50:50]   |
V10Log.GPS.lat = GPS(:,4);                             %   3. | lat     | lat                           | [D]   | [50:50]   |
V10Log.GPS.lon = GPS(:,5);                             %   4. | lon     | lon                           | [D]   | [50:50]   |
V10Log.GPS.height = GPS(:,6);                          %   5. | hgt     | height                        | [D]   | [50:50]   |
V10Log.GPS.HDOP = GPS(:,15);                           %   6. | hdop    | HDOP                          | [F]   | [50:50]   |
V10Log.GPS.PDOP = GPS(:,17);                           %   7. | pdop    | PDOP                          | [F]   | [50:50]   |
%% GPSE
V10Log.GPSE.TimeUS = GPSE(:,2);                        %   1. | TimeUS  | TimeUS                        | [U64] | [50:50]   |
V10Log.GPSE.CNT = GPSE(:,3);                           %   2. | CNT     | CNT                           | [U32] | [50:50]   |
V10Log.GPSE.decode_error_cnt = GPSE(:,4);              %   3. | err1    | decode_error_cnt              | [U32] | [50:50]   |
V10Log.GPSE.decode_crc_error_cnt = GPSE(:,5);          %   4. | err2    | decode_crc_error_cnt          | [U32] | [50:50]   |
V10Log.GPSE.decode_psrdop_cnt = GPSE(:,6);             %   5. | err3    | decode_psrdop_cnt             | [U32] | [50:50]   |
V10Log.GPSE.oem718d_decode_status = GPSE(:,7);         %   6. | err4    | oem718d_decode_status         | [U32] | [50:50]   |
V10Log.GPSE.decode_bestpos_cnt = GPSE(:,8);            %   7. | err5    | decode_bestpos_cnt            | [U32] | [50:50]   |
V10Log.GPSE.decode_bestvel_cnt = GPSE(:,9);            %   8. | err6    | decode_bestvel_cnt            | [U32] | [50:50]   |
V10Log.GPSE.decode_heading_cnt = GPSE(:,10);           %   9. | err7    | decode_heading_cnt            | [U32] | [50:50]   |
V10Log.GPSE.rev = GPSE(:,11);                          %  10. | err8    | rev                           | [U32] | [50:50]   |
V10Log.GPSE.Sum = GPSE(:,12);                          %  11. | Sum     | Sum                           | [U8]  | [50:50]   |
%% UBX
V10Log.UBX.TimeUS = UBX(:,2);                          %   1. | TimeUS  | TimeUS                        | [U64] | [100:100] |
V10Log.UBX.CNT = UBX(:,3);                             %   2. | CNT     | CNT                           | [U32] | [100:100] |
V10Log.UBX.lat = UBX(:,4);                             %   3. | lat     | lat                           | [I32] | [100:100] |
V10Log.UBX.lon = UBX(:,5);                             %   4. | lon     | lon                           | [I32] | [100:100] |
V10Log.UBX.height = UBX(:,6);                          %   5. | hgt     | height                        | [I32] | [100:100] |
V10Log.UBX.velN = UBX(:,14);                           %   6. | velN    | velN                          | [I32] | [100:100] |
V10Log.UBX.velE = UBX(:,16);                           %   7. | velE    | velE                          | [I32] | [100:100] |
%% UBXE
V10Log.UBXE.TimeUS = UBXE(:,2);                        %   1. | TimeUS  | TimeUS                        | [U64] | [100:100] |
V10Log.UBXE.CNT = UBXE(:,3);                           %   2. | CNT     | CNT                           | [U32] | [100:100] |
V10Log.UBXE.decode_error_cnt = UBXE(:,4);              %   3. | err1    | decode_error_cnt              | [U32] | [100:100] |
V10Log.UBXE.decode_crc_error_cnt = UBXE(:,4);          %   4. | err1    | decode_crc_error_cnt          | [U32] | [100:100] |
V10Log.UBXE.decode_psrdop_cnt = UBXE(:,4);             %   5. | err1    | decode_psrdop_cnt             | [U32] | [100:100] |
V10Log.UBXE.nak_error = UBXE(:,4);                     %   6. | err1    | nak_error                     | [U32] | [100:100] |
V10Log.UBXE.rev1 = UBXE(:,4);                          %   7. | err1    | rev1                          | [U32] | [100:100] |
V10Log.UBXE.rev2 = UBXE(:,4);                          %   8. | err1    | rev2                          | [U32] | [100:100] |
V10Log.UBXE.rev3 = UBXE(:,4);                          %   9. | err1    | rev3                          | [U32] | [100:100] |
V10Log.UBXE.rev4 = UBXE(:,4);                          %  10. | err1    | rev4                          | [U32] | [100:100] |
V10Log.UBXE.Sum = UBXE(:,12);                          %  11. | Sum     | Sum                           | [U8]  | [100:100] |
%% RCIN
V10Log.RCIN.TimeUS = RCIN(:,2);                        %   1. | TimeUS  | TimeUS                        | [U64] | [7:7]     |
V10Log.RCIN.recv_total_cnt = RCIN(:,3);                %   2. | CNT     | recv_total_cnt                | [U32] | [7:7]     |
V10Log.RCIN.frame_lost_cnt = RCIN(:,4);                %   3. | LCNT    | frame_lost_cnt                | [U32] | [7:7]     |
V10Log.RCIN.channel_1_roll = RCIN(:,5);                %   4. | C1      | channel_1_roll                | [U16] | [7:7]     |
V10Log.RCIN.channel_2_pitch = RCIN(:,6);               %   5. | C2      | channel_2_pitch               | [U16] | [7:7]     |
V10Log.RCIN.channel_3_throttle = RCIN(:,7);            %   6. | C3      | channel_3_throttle            | [U16] | [7:7]     |
V10Log.RCIN.channel_4_yaw = RCIN(:,8);                 %   7. | C4      | channel_4_yaw                 | [U16] | [7:7]     |
V10Log.RCIN.channel_5_Auto = RCIN(:,9);                %   8. | C5      | channel_5_Auto                | [U16] | [7:7]     |
V10Log.RCIN.channel_6_tilt = RCIN(:,10);               %   9. | C6      | channel_6_tilt                | [U16] | [7:7]     |
V10Log.RCIN.channel_7_D = RCIN(:,11);                  %  10. | C7      | channel_7_D                   | [U16] | [7:7]     |
V10Log.RCIN.channel_8_C = RCIN(:,12);                  %  11. | C8      | channel_8_C                   | [U16] | [7:7]     |
V10Log.RCIN.channel_9_Lock = RCIN(:,13);               %  12. | C9      | channel_9_Lock                | [U16] | [7:7]     |
V10Log.RCIN.channel_F = RCIN(:,14);                    %  13. | C10     | channel_F                     | [U16] | [7:7]     |
V10Log.RCIN.Sum = RCIN(:,15);                          %  14. | Sum     | Sum                           | [U8]  | [7:7]     |
%% PWMO
V10Log.PWMO.TimeUS = PWMO(:,2);                        %   1. | TimeUS  | TimeUS                        | [U64] | [12:12]   |
V10Log.PWMO.count = PWMO(:,3);                         %   2. | CNT     | count                         | [U32] | [12:12]   |
V10Log.PWMO.lost_cnt = PWMO(:,4);                      %   3. | LCNT    | lost_cnt                      | [U32] | [12:12]   |
V10Log.PWMO.start_count = PWMO(:,5);                   %   4. | SCNT    | start_count                   | [U16] | [12:12]   |
V10Log.PWMO.pwm_esc_0 = PWMO(:,6);                     %   5. | EC0     | pwm_esc_0                     | [U16] | [12:12]   |
V10Log.PWMO.pwm_esc_1 = PWMO(:,7);                     %   6. | EC1     | pwm_esc_1                     | [U16] | [12:12]   |
V10Log.PWMO.pwm_esc_2 = PWMO(:,8);                     %   7. | EC2     | pwm_esc_2                     | [U16] | [12:12]   |
V10Log.PWMO.pwm_esc_3 = PWMO(:,9);                     %   8. | EC3     | pwm_esc_3                     | [U16] | [12:12]   |
V10Log.PWMO.pwm_esc_4 = PWMO(:,10);                    %   9. | EC4     | pwm_esc_4                     | [U16] | [12:12]   |
V10Log.PWMO.pwm_servo_0 = PWMO(:,11);                  %  10. | SV0     | pwm_servo_0                   | [U16] | [12:12]   |
V10Log.PWMO.pwm_servo_1 = PWMO(:,12);                  %  11. | SV1     | pwm_servo_1                   | [U16] | [12:12]   |
V10Log.PWMO.pwm_servo_2 = PWMO(:,13);                  %  12. | SV2     | pwm_servo_2                   | [U16] | [12:12]   |
V10Log.PWMO.pwm_servo_3 = PWMO(:,14);                  %  13. | SV3     | pwm_servo_3                   | [U16] | [12:12]   |
V10Log.PWMO.pwm_servo_4 = PWMO(:,15);                  %  14. | SV4     | pwm_servo_4                   | [U16] | [12:12]   |
V10Log.PWMO.Sum = PWMO(:,16);                          %  15. | Sum     | Sum                           | [U8]  | [12:12]   |
%% ALG2
V10Log.ALG2.TimeUS = ALG2(:,2);                        %   1. | TimeUS  | TimeUS                        | [U64] | [12:12]   |
V10Log.ALG2.count = ALG2(:,3);                         %   2. | CNT     | count                         | [U32] | [12:12]   |
V10Log.ALG2.exe_time = ALG2(:,4);                      %   3. | exe     | exe_time                      | [U64] | [12:12]   |
V10Log.ALG2.algo_remot_roll = ALG2(:,5);               %   4. | rol     | algo_remot_roll               | [F]   | [12:12]   |
V10Log.ALG2.algo_remot_pitch = ALG2(:,6);              %   5. | pit     | algo_remot_pitch              | [F]   | [12:12]   |
V10Log.ALG2.algo_remot_yaw = ALG2(:,7);                %   6. | yaw     | algo_remot_yaw                | [F]   | [12:12]   |
V10Log.ALG2.algo_remot_throttle = ALG2(:,8);           %   7. | thr     | algo_remot_throttle           | [F]   | [12:12]   |
V10Log.ALG2.algo_remot_tilt_angle_in = ALG2(:,9);      %   8. | tilt    | algo_remot_tilt_angle_in      | [F]   | [12:12]   |
V10Log.ALG2.Sum = ALG2(:,10);                          %   9. | Sum     | Sum                           | [U8]  | [12:12]   |
%% ALD1
V10Log.ALD1.algo_TimeUS = ALD1(:,2);                        %   1. | Tim     | TimeUS                        | [U64] | [12:12]   |
V10Log.ALD1.algo_CNT = ALD1(:,3);                           %   2. | CNT     | CNT                           | [U32] | [12:12]   |
V10Log.ALD1.algo_time = ALD1(:,4);                          %   3. | time    | time                          | [U64] | [12:12]   |
V10Log.ALD1.algo_mode = ALD1(:,6);                          %   4. | mode    | mode                          | [U8]  | [12:12]   |
V10Log.ALD1.algo_palne_mode = ALD1(:,7);                    %   5. | plmod   | palne_mode                    | [U8]  | [12:12]   |
V10Log.ALD1.algo_flightTaskMode = ALD1(:,8);                %   6. | fmod    | flightTaskMode                | [U8]  | [12:12]   |
V10Log.ALD1.algo_PathMode = ALD1(:,9);                      %   7. | ptmod   | PathMode                      | [U8]  | [12:12]   |
V10Log.ALD1.algo_limit_pos_up = ALD1(:,10);                 %   8. | posup   | limit_pos_up                  | [U8]  | [12:12]   |
V10Log.ALD1.algo_throttle_upper = ALD1(:,11);               %   9. | thup    | throttle_upper                | [U8]  | [12:12]   |
V10Log.ALD1.algo_throttle_lower = ALD1(:,12);               %  10. | thlo    | throttle_lower                | [U8]  | [12:12]   |
V10Log.ALD1.algo_Sum = ALD1(:,13);                          %  11. | Sum     | Sum                           | [U8]  | [12:12]   |
%% ALD2
V10Log.ALD1.algo_TimeUS = ALD2(:,2);                        %   1. | Tim     | TimeUS                        | [U64] | [12:12]   |
V10Log.ALD1.algo_CNT = ALD2(:,3);                           %   2. | CNT     | CNT                           | [U32] | [12:12]   |
V10Log.ALD1.algo_roll = ALD2(:,4)*HD;                          %   3. | rol     | roll                          | [F]   | [12:12]   |
V10Log.ALD1.algo_pitch = ALD2(:,5)*HD;                         %   4. | pit     | pitch                         | [F]   | [12:12]   |
V10Log.ALD1.algo_yaw = ALD2(:,6)*HD;                           %   5. | yaw     | yaw                           | [F]   | [12:12]   |
V10Log.ALD1.algo_roll_in = ALD2(:,7);                       %   6. | rin     | roll_in                       | [F]   | [12:12]   |
V10Log.ALD1.algo_pitch_in = ALD2(:,8);                      %   7. | pin     | pitch_in                      | [F]   | [12:12]   |
V10Log.ALD1.algo_yaw_in = ALD2(:,9);                        %   8. | yin     | yaw_in                        | [F]   | [12:12]   |
V10Log.ALD1.algo_throttle_in = ALD2(:,10);                  %   9. | tin     | throttle_in                   | [F]   | [12:12]   |
V10Log.ALD1.algo_pwm_out_1 = ALD2(:,11);                    %  10. | pw0     | pwm_out_0                     | [F]   | [12:12]   |
V10Log.ALD1.algo_pwm_out_2 = ALD2(:,12);                    %  11. | pw1     | pwm_out_1                     | [F]   | [12:12]   |
V10Log.ALD1.algo_pwm_out_3 = ALD2(:,13);                    %  12. | pw2     | pwm_out_2                     | [F]   | [12:12]   |
V10Log.ALD1.algo_pwm_out_4 = ALD2(:,14);                    %  13. | pw3     | pwm_out_3                     | [F]   | [12:12]   |
V10Log.ALD1.algo_tail_tilt = ALD2(:,15);                    %  14. | til     | tail_tilt                     | [F]   | [12:12]   |
V10Log.ALD1.algo_pwm_tail = ALD2(:,16);                     %  15. | pwm     | pwm_tail                      | [F]   | [12:12]   |
V10Log.ALD1.algo_Sum = ALD2(:,17);                          %  16. | Sm      | Sum                           | [F]   | [12:12]   |
%% ALD3
V10Log.ALD1.algo_TimeUS = ALD3(:,2);                        %   1. | Tim     | TimeUS                        | [U64] | [12:12]   |
V10Log.ALD1.algo_CNT = ALD3(:,3);                           %   2. | CNT     | CNT                           | [U32] | [12:12]   |
V10Log.ALD1.algo_yaw_out = ALD3(:,4);                       %   3. | yout    | yaw_out                       | [F]   | [12:12]   |
V10Log.ALD1.algo_k_flap = ALD3(:,5);                        %   4. | flap    | k_flap                        | [F]   | [12:12]   |
V10Log.ALD1.algo_current_loc_0 = ALD3(:,6);                 %   5. | cl0     | current_loc_0                 | [F]   | [12:12]   |
V10Log.ALD1.algo_current_loc_1 = ALD3(:,7);                 %   6. | cl1     | current_loc_1                 | [F]   | [12:12]   |
V10Log.ALD1.algo_curr_vel_0 = ALD3(:,8);                    %   7. | cv0     | curr_vel_0                    | [F]   | [12:12]   |
V10Log.ALD1.algo_curr_vel_1 = ALD3(:,9);                    %   8. | cv1     | curr_vel_1                    | [F]   | [12:12]   |
V10Log.ALD1.algo_curr_vel_2 = ALD3(:,10);                   %   9. | cv2     | curr_vel_2                    | [F]   | [12:12]   |
V10Log.ALD1.algo_curr_pos_0 = ALD3(:,11);                   %  10. | cp0     | curr_pos_0                    | [F]   | [12:12]   |
V10Log.ALD1.algo_curr_pos_1 = ALD3(:,12);                   %  11. | cp1     | curr_pos_1                    | [F]   | [12:12]   |
V10Log.ALD1.algo_rate_target_ang_vel_0 = ALD3(:,13);        %  12. | rvl0    | rate_target_ang_vel_0         | [F]   | [12:12]   |
V10Log.ALD1.algo_rate_target_ang_vel_1 = ALD3(:,14);        %  13. | rvl1    | rate_target_ang_vel_1         | [F]   | [12:12]   |
V10Log.ALD1.algo_rate_target_ang_vel_2 = ALD3(:,15);        %  14. | rvl2    | rate_target_ang_vel_2         | [F]   | [12:12]   |
V10Log.ALD1.algo_Sum = ALD3(:,16);                          %  15. | Sm      | Sum                           | [F]   | [12:12]   |
%% ALD4
V10Log.ALD1.algo_TimeUS = ALD4(:,2);                        %   1. | Tim     | TimeUS                        | [U64] | [12:12]   |
V10Log.ALD1.algo_CNT = ALD4(:,3);                           %   2. | CNT     | CNT                           | [U32] | [12:12]   |
V10Log.ALD1.algo_attitude_target_euler_rate_0 = ALD4(:,4);  %   3. | er0     | attitude_target_euler_rate_0  | [F]   | [12:12]   |
V10Log.ALD1.algo_attitude_target_euler_rate_1 = ALD4(:,5);  %   4. | er1     | attitude_target_euler_rate_1  | [F]   | [12:12]   |
V10Log.ALD1.algo_attitude_target_euler_rate_2 = ALD4(:,6);  %   5. | er2     | attitude_target_euler_rate_2  | [F]   | [12:12]   |
V10Log.ALD1.algo_attitude_target_euler_angle_0 = ALD4(:,7)*HD; %   6. | ea0     | attitude_target_euler_angle_0 | [F]   | [12:12]   |
V10Log.ALD1.algo_attitude_target_euler_angle_1 = ALD4(:,8)*HD; %   7. | ea1     | attitude_target_euler_angle_1 | [F]   | [12:12]   |
V10Log.ALD1.algo_attitude_target_euler_angle_2 = ALD4(:,9)*HD; %   8. | ea2     | attitude_target_euler_angle_2 | [F]   | [12:12]   |
V10Log.ALD1.algo_pos_target_0 = ALD4(:,10);                 %   9. | ptg0    | pos_target_0                  | [F]   | [12:12]   |
V10Log.ALD1.algo_pos_target_1 = ALD4(:,11);                 %  10. | ptg1    | pos_target_1                  | [F]   | [12:12]   |
V10Log.ALD1.algo_pos_target_2 = ALD4(:,12);                 %  11. | ptg2    | pos_target_2                  | [F]   | [12:12]   |
V10Log.ALD1.algo_vel_target_0 = ALD4(:,13);                 %  12. | vtg0    | vel_target_0                  | [F]   | [12:12]   |
V10Log.ALD1.algo_vel_target_1 = ALD4(:,14);                 %  13. | vtg1    | vel_target_1                  | [F]   | [12:12]   |
V10Log.ALD1.algo_vel_target_2 = ALD4(:,15);                 %  14. | vtg2    | vel_target_2                  | [F]   | [12:12]   |
V10Log.ALD1.algo_Sum = ALD4(:,16);                          %  15. | Sm      | Sum                           | [F]   | [12:12]   |
%% ALD5
V10Log.ALD1.algo_TimeUS = ALD5(:,2);                        %   1. | Tim     | TimeUS                        | [U64] | [12:12]   |
V10Log.ALD1.algo_CNT = ALD5(:,3);                           %   2. | CNT     | CNT                           | [U32] | [12:12]   |
V10Log.ALD1.algo_accel_target_0 = ALD5(:,4);                %   3. | acc0    | accel_target_0                | [F]   | [12:12]   |
V10Log.ALD1.algo_accel_target_1 = ALD5(:,5);                %   4. | acc1    | accel_target_1                | [F]   | [12:12]   |
V10Log.ALD1.algo_accel_target_2 = ALD5(:,6);                %   5. | acc2    | accel_target_2                | [F]   | [12:12]   |
V10Log.ALD1.algo_attitude_error_vector_0 = ALD5(:,7);       %   6. | vect0   | attitude_error_vector_0       | [F]   | [12:12]   |
V10Log.ALD1.algo_attitude_error_vector_1 = ALD5(:,8);       %   7. | vect1   | attitude_error_vector_1       | [F]   | [12:12]   |
V10Log.ALD1.algo_attitude_error_vector_2 = ALD5(:,9);       %   8. | vect2   | attitude_error_vector_2       | [F]   | [12:12]   |
V10Log.ALD1.algo_pos_error_0 = ALD5(:,10);                  %   9. | per0    | pos_error_0                   | [F]   | [12:12]   |
V10Log.ALD1.algo_pos_error_1 = ALD5(:,11);                  %  10. | per1    | pos_error_1                   | [F]   | [12:12]   |
V10Log.ALD1.algo_pos_error_2 = ALD5(:,12);                  %  11. | per2    | pos_error_2                   | [F]   | [12:12]   |
V10Log.ALD1.algo_vel_desired_2 = ALD5(:,13);                %  12. | veld2   | vel_desired_2                 | [F]   | [12:12]   |
V10Log.ALD1.algo_Sum = ALD5(:,14);                          %  13. | Sm      | Sum                           | [F]   | [12:12]   |
%% ALD6
V10Log.ALD1.algo_TimeUS = ALD6(:,2);                        %   1. | Tim     | TimeUS                        | [U64] | [12:12]   |
V10Log.ALD1.algo_CNT = ALD6(:,3);                           %   2. | CNT     | CNT                           | [U32] | [12:12]   |
V10Log.ALD1.algo_z_accel_meas = ALD6(:,4);                  %   3. | zacm    | z_accel_meas                  | [F]   | [12:12]   |
V10Log.ALD1.algo_climb_rate_cms = ALD6(:,5);                %   4. | clrc    | climb_rate_cms                | [F]   | [12:12]   |
V10Log.ALD1.algo_throttle_filter = ALD6(:,6);               %   5. | tfil    | throttle_filter               | [F]   | [12:12]   |
V10Log.ALD1.algo_nav_pitch_cd = ALD6(:,7);                  %   6. | navcd   | nav_pitch_cd                  | [F]   | [12:12]   |
V10Log.ALD1.algo_vel_forward_last_pct = ALD6(:,8);          %   7. | vlpct   | vel_forward_last_pct          | [F]   | [12:12]   |
V10Log.ALD1.algo_k_rudder = ALD6(:,9);                      %   8. | krud    | k_rudder                      | [F]   | [12:12]   |
V10Log.ALD1.algo_k_elevator = ALD6(:,10);                   %   9. | kele    | k_elevator                    | [F]   | [12:12]   |
V10Log.ALD1.algo_k_throttle = ALD6(:,11);                   %  10. | kthr    | k_throttle                    | [F]   | [12:12]   |
V10Log.ALD1.algo_k_aileron = ALD6(:,12);                    %  11. | kail    | k_aileron                     | [F]   | [12:12]   |
V10Log.ALD1.algo_curr_alt = ALD6(:,13)/100;                     %  12. | curalt  | curr_alt                      | [F]   | [12:12]   |
V10Log.ALD1.algo_Sum = ALD6(:,14);                          %  13. | Sm      | Sum                           | [F]   | [12:12]   |
%% ALD7
V10Log.ALD1.algo_TimeUS = ALD7(:,2);                        %   1. | Tim     | TimeUS                        | [U64] | [12:12]   |
V10Log.ALD1.algo_CNT = ALD7(:,3);                           %   2. | CNT     | CNT                           | [U32] | [12:12]   |
V10Log.ALD1.algo_weathervane_last_output = ALD7(:,4);       %   3. | weat    | weathervane_last_output       | [F]   | [12:12]   |
V10Log.ALD1.algo_roll_target = ALD7(:,5)/100;                   %   4. | rotg    | roll_target                   | [F]   | [12:12]   |
V10Log.ALD1.algo_pitch_target = ALD7(:,6)/100;                  %   5. | pitg    | pitch_target                  | [F]   | [12:12]   |
V10Log.ALD1.algo_roll_target_pilot = ALD7(:,7);             %   6. | rotp    | roll_target_pilot             | [F]   | [12:12]   |
V10Log.ALD1.algo_pitch_dem = ALD7(:,8);                     %   7. | pitdm   | pitch_dem                     | [F]   | [12:12]   |
V10Log.ALD1.algo_hgt_dem = ALD7(:,9);                       %   8. | hgtdm   | hgt_dem                       | [F]   | [12:12]   |
V10Log.ALD1.algo_throttle_dem = ALD7(:,10);                 %   9. | thdm    | throttle_dem                  | [F]   | [12:12]   |
V10Log.ALD1.algo_latAccDem = ALD7(:,11);                    %  10. | accdm   | latAccDem                     | [F]   | [12:12]   |
V10Log.ALD1.algo_aspeed = ALD7(:,12);                       %  11. | aspd    | aspeed                        | [F]   | [12:12]   |
V10Log.ALD1.algo_pitch_target_pilot = ALD7(:,13);           %  12. | pitpi   | pitch_target_pilot            | [F]   | [12:12]   |
V10Log.ALD1.algo_Sum = ALD7(:,14);                          %  13. | Sm      | Sum                           | [F]   | [12:12]   |
%% ALD8
V10Log.ALD8.algo_TimeUS = ALD8(:,2);                        %   1. | Tim     | TimeUS                        | [U64] | [12:12]   |
V10Log.ALD8.algo_CNT = ALD8(:,3);                           %   2. | CNT     | CNT                           | [U32] | [12:12]   |
V10Log.ALD8.algo_WP_i = ALD8(:,4);                          %   3. | WP_i    | WP_i                          | [F]   | [12:12]   |
V10Log.ALD8.algo_sl_heightCmd = ALD8(:,5);                  %   4. | hgtcmd  | sl_heightCmd                  | [F]   | [12:12]   |
V10Log.ALD8.algo_sl_maxClimbSpeed = ALD8(:,6);              %   5. | clmspd  | sl_maxClimbSpeed              | [F]   | [12:12]   |
V10Log.ALD8.algo_sl_flightTaskMode = ALD8(:,7);             %   6. | fmode   | sl_flightTaskMode             | [F]   | [12:12]   |
V10Log.ALD8.algo_Sum = ALD8(:,8);                           %   7. | Sm      | Sum                           | [F]   | [12:12]   |
%% ALL1
V10Log.IN_MAVLINK.TimeUS = ALL1(:,2);                  %   1. | Tim     | TimeUS                        | [U64] | [12:12]   |
V10Log.IN_MAVLINK.CNT = ALL1(:,3);                     %   2. | CNT     | CNT                           | [U32] | [12:12]   |
V10Log.IN_MAVLINK.mavlink_msg_groundHomeLLA_0 = ALL1(:,4); %   3. | LLA0    | mavlink_msg_groundHomeLLA_0   | [F]   | [12:12]   |
V10Log.IN_MAVLINK.mavlink_msg_groundHomeLLA_1 = ALL1(:,5); %   4. | LLA1    | mavlink_msg_groundHomeLLA_1   | [F]   | [12:12]   |
V10Log.IN_MAVLINK.mavlink_msg_groundHomeLLA_2 = ALL1(:,6); %   5. | LLA2    | mavlink_msg_groundHomeLLA_2   | [F]   | [12:12]   |
% %% ALL2
% V10Log.IN_MAVLINK.mavlink_msg_command_battery_data.TimeUS = ALL2(:,3); %   1. | Tim     | TimeUS                        | [U64] | [12:12]   |
% V10Log.IN_MAVLINK.mavlink_msg_command_battery_data.CNT = ALL2(:,4); %   2. | CNT     | CNT                           | [U32] | [12:12]   |
% V10Log.IN_MAVLINK.mavlink_msg_command_battery_data.fullCapacity = ALL2(:,5); %   3. | cap     | fullCapacity                  | [U16] | [12:12]   |
% V10Log.IN_MAVLINK.mavlink_msg_command_battery_data.lifePercent = ALL2(:,6); %   4. | life    | lifePercent                   | [U8]  | [12:12]   |
% V10Log.IN_MAVLINK.mavlink_msg_command_battery_data.cycleTime = ALL2(:,7); %   5. | cycle   | cycleTime                     | [U16] | [12:12]   |
% V10Log.IN_MAVLINK.mavlink_msg_command_battery_data.batteryId = ALL2(:,8); %   6. | batID   | batteryId                     | [U16] | [12:12]   |
% %% ALL3
% V10Log.IN_MAVLINK.mavlink_mission_item_def.TimeUS = ALL3(:,2); %   1. | Tim     | TimeUS                        | [U64] | [12:12]   |
% V10Log.IN_MAVLINK.mavlink_mission_item_def.CNT = ALL3(:,3); %   2. | CNT     | CNT                           | [U32] | [12:12]   |
% V10Log.IN_MAVLINK.mavlink_mission_item_def.seq = ALL3(:,4); %   3. | seq     | seq                           | [U16] | [12:12]   |
% V10Log.IN_MAVLINK.mavlink_mission_item_def.x = ALL3(:,5); %   4. | x       | x                             | [F]   | [12:12]   |
% V10Log.IN_MAVLINK.mavlink_mission_item_def.y = ALL3(:,6); %   5. | y       | y                             | [F]   | [12:12]   |
% V10Log.IN_MAVLINK.mavlink_mission_item_def.z = ALL3(:,7); %   6. | z       | z                             | [F]   | [12:12]   |
% V10Log.IN_MAVLINK.mavlink_mission_item_def.Sum = ALL3(:,8); %   7. | Sm      | Sum                           | [U8]  | [12:12]   |
%% ALL4
V10Log.SensorSelect.TimeUS = ALL4(:,2);                %   1. | Tim     | TimeUS                        | [U64] | [12:12]   |
V10Log.SensorSelect.CNT = ALL4(:,3);                   %   2. | CNT     | CNT                           | [U32] | [12:12]   |
V10Log.SensorSelect.IMU = ALL4(:,4);                   %   3. | IMU     | IMU                           | [F]   | [12:12]   |
V10Log.SensorSelect.Mag = ALL4(:,5);                   %   4. | Mag     | Mag                           | [F]   | [12:12]   |
V10Log.SensorSelect.GPS = ALL4(:,6);                   %   5. | GPS     | GPS                           | [F]   | [12:12]   |
V10Log.SensorSelect.Baro = ALL4(:,7);                  %   6. | Bar     | Baro                          | [F]   | [12:12]   |
V10Log.SensorSelect.Radar = ALL4(:,8);                 %   7. | Radr    | Radar                         | [F]   | [12:12]   |
V10Log.SensorSelect.Camera = ALL4(:,9);                %   8. | CAM     | Camera                        | [F]   | [12:12]   |
V10Log.SensorSelect.Lidar = ALL4(:,10);                %   9. | Lidr    | Lidar                         | [F]   | [12:12]   |
%% ALL5
V10Log.SensorUpdateFlag.TimeUS = ALL5(:,2);            %   1. | Tim     | TimeUS                        | [U64] | [12:12]   |
V10Log.SensorUpdateFlag.CNT = ALL5(:,3);               %   2. | CNT     | CNT                           | [U32] | [12:12]   |
V10Log.SensorUpdateFlag.mag1 = ALL5(:,4);              %   3. | mg1     | mag1                          | [U8]  | [12:12]   |
V10Log.SensorUpdateFlag.mag2 = ALL5(:,5);              %   4. | mg2     | mag2                          | [U8]  | [12:12]   |
V10Log.SensorUpdateFlag.airspeed1 = ALL5(:,6);         %   5. | ap1     | airspeed1                     | [U8]  | [12:12]   |
V10Log.SensorUpdateFlag.airspeed2 = ALL5(:,7);         %   6. | ap2     | airspeed2                     | [U8]  | [12:12]   |
V10Log.SensorUpdateFlag.baro1 = ALL5(:,8);             %   7. | br1     | baro1                         | [U8]  | [12:12]   |
V10Log.SensorUpdateFlag.baro2 = ALL5(:,9);             %   8. | br2     | baro2                         | [U8]  | [12:12]   |
V10Log.SensorUpdateFlag.IMU1 = ALL5(:,10);             %   9. | m1      | IMU1                          | [U8]  | [12:12]   |
V10Log.SensorUpdateFlag.IMU2 = ALL5(:,11);             %  10. | m2      | IMU2                          | [U8]  | [12:12]   |
V10Log.SensorUpdateFlag.IMU3 = ALL5(:,12);             %  11. | m3      | IMU3                          | [U8]  | [12:12]   |
V10Log.SensorUpdateFlag.IMU4 = ALL5(:,13);             %  12. | m4      | IMU4                          | [U8]  | [12:12]   |
V10Log.SensorUpdateFlag.um482 = ALL5(:,14);            %  13. | um482   | um482                         | [U8]  | [12:12]   |
V10Log.SensorUpdateFlag.ublox1 = ALL5(:,15);           %  14. | ubx1    | ublox1                        | [U8]  | [12:12]   |
V10Log.SensorUpdateFlag.radar1 = ALL5(:,16);           %  15. | radr1   | radar1                        | [U8]  | [12:12]   |
V10Log.SensorUpdateFlag.Sum = ALL5(:,17);              %  16. | Sm      | Sum                           | [U8]  | [12:12]   |
%% ALL6
V10Log.SensorLosttime.TimeUS = ALL6(:,2);              %   1. | Tim     | TimeUS                        | [U64] | [12:12]   |
V10Log.SensorLosttime.CNT = ALL6(:,3);                 %   2. | CNT     | CNT                           | [U32] | [12:12]   |
V10Log.SensorLosttime.mag1 = ALL6(:,4);                %   3. | mg1     | mag1                          | [F]   | [12:12]   |
V10Log.SensorLosttime.mag2 = ALL6(:,5);                %   4. | mg2     | mag2                          | [F]   | [12:12]   |
V10Log.SensorLosttime.airspeed1 = ALL6(:,6);           %   5. | ap1     | airspeed1                     | [F]   | [12:12]   |
V10Log.SensorLosttime.airspeed2 = ALL6(:,7);           %   6. | ap2     | airspeed2                     | [F]   | [12:12]   |
V10Log.SensorLosttime.baro1 = ALL6(:,8);               %   7. | br1     | baro1                         | [F]   | [12:12]   |
V10Log.SensorLosttime.baro2 = ALL6(:,9);               %   8. | br2     | baro2                         | [F]   | [12:12]   |
V10Log.SensorLosttime.IMU1 = ALL6(:,10);               %   9. | m1      | IMU1                          | [F]   | [12:12]   |
V10Log.SensorLosttime.IMU2 = ALL6(:,11);               %  10. | m2      | IMU2                          | [F]   | [12:12]   |
V10Log.SensorLosttime.IMU3 = ALL6(:,12);               %  11. | m3      | IMU3                          | [F]   | [12:12]   |
V10Log.SensorLosttime.IMU4 = ALL6(:,13);               %  12. | m4      | IMU4                          | [F]   | [12:12]   |
V10Log.SensorLosttime.um482 = ALL6(:,14);              %  13. | um482   | um482                         | [F]   | [12:12]   |
V10Log.SensorLosttime.ublox1 = ALL6(:,15);             %  14. | ubx1    | ublox1                        | [F]   | [12:12]   |
V10Log.SensorLosttime.radar1 = ALL6(:,16);             %  15. | radr1   | radar1                        | [F]   | [12:12]   |
V10Log.SensorLosttime.Sum = ALL6(:,17);                %  16. | Sm      | Sum                           | [U8]  | [12:12]   |
%% ALL8
V10Log.CAMERA.TimeUS = ALL8(:,2);                      %   1. | Tim     | TimeUS                        | [U64] | [12:12]   |
V10Log.CAMERA.CNT = ALL8(:,3);                         %   2. | CNT     | CNT                           | [U32] | [12:12]   |
V10Log.CAMERA.time = ALL8(:,4);                        %   3. | ctim    | time                          | [F]   | [12:12]   |
V10Log.CAMERA.trigger = ALL8(:,5);                     %   4. | ctrg    | trigger                       | [F]   | [12:12]   |
V10Log.CAMERA.LLA_0 = ALL8(:,6);                       %   5. | cLL0    | LLA_0                         | [F]   | [12:12]   |
V10Log.CAMERA.LLA_1 = ALL8(:,7);                       %   6. | cLL1    | LLA_1                         | [F]   | [12:12]   |
V10Log.CAMERA.LLA_2 = ALL8(:,8);                       %   7. | cLL2    | LLA_2                         | [F]   | [12:12]   |
V10Log.CAMERA.groundspeed = ALL8(:,9);                 %   8. | gspd    | groundspeed                   | [F]   | [12:12]   |
V10Log.CAMERA.Sum = ALL8(:,10);                        %   9. | Sm      | Sum                           | [U8]  | [12:12]   |
%% ALL9
V10Log.LIDAR.TimeUS = ALL9(:,2);                       %   1. | Tim     | TimeUS                        | [U64] | [12:12]   |
V10Log.LIDAR.CNT = ALL9(:,3);                          %   2. | CNT     | CNT                           | [U32] | [12:12]   |
V10Log.LIDAR.time = ALL9(:,4);                         %   3. | dtim    | time                          | [F]   | [12:12]   |
V10Log.LIDAR.trigger = ALL9(:,5);                      %   4. | dtrg    | trigger                       | [F]   | [12:12]   |
V10Log.LIDAR.LLA_0 = ALL9(:,6);                        %   5. | dLL0    | LLA_0                         | [F]   | [12:12]   |
V10Log.LIDAR.LLA_1 = ALL9(:,7);                        %   6. | dLL1    | LLA_1                         | [F]   | [12:12]   |
V10Log.LIDAR.LLA_2 = ALL9(:,8);                        %   7. | dLL2    | LLA_2                         | [F]   | [12:12]   |
V10Log.LIDAR.groundspeed = ALL9(:,9);                  %   8. | gspd    | groundspeed                   | [F]   | [12:12]   |
V10Log.LIDAR.Sum = ALL9(:,10);                         %   9. | Sm      | Sum                           | [U8]  | [12:12]   |
%% AL10
V10Log.PowerConsume.TimeUS = AL10(:,2);                %   1. | Tim     | TimeUS                        | [U64] | [12:12]   |
V10Log.PowerConsume.CNT = AL10(:,3);                   %   2. | CNT     | CNT                           | [U32] | [12:12]   |
V10Log.PowerConsume.AllTheTimeVoltage = AL10(:,4);     %   3. | vol     | AllTheTimeVoltage             | [U8]  | [12:12]   |
V10Log.PowerConsume.AllTheTimeCurrent = AL10(:,5);     %   4. | cur     | AllTheTimeCurrent             | [U8]  | [12:12]   |
V10Log.PowerConsume.AllTheTimePowerConsume = AL10(:,6); %   5. | pow     | AllTheTimePowerConsume        | [U8]  | [12:12]   |
V10Log.PowerConsume.GroundStandby = AL10(:,7);         %   6. | std     | GroundStandby                 | [U8]  | [12:12]   |
V10Log.PowerConsume.TakeOff = AL10(:,8);               %   7. | take    | TakeOff                       | [U8]  | [12:12]   |
V10Log.PowerConsume.HoverAdjust = AL10(:,9);           %   8. | hva     | HoverAdjust                   | [U8]  | [12:12]   |
V10Log.PowerConsume.Rotor2fix = AL10(:,10);            %   9. | rot     | Rotor2fix                     | [U8]  | [12:12]   |
V10Log.PowerConsume.HoverUp = AL10(:,11);              %  10. | hvp     | HoverUp                       | [U8]  | [12:12]   |
V10Log.PowerConsume.PathFollow = AL10(:,12);           %  11. | pth     | PathFollow                    | [U8]  | [12:12]   |
V10Log.PowerConsume.GoHome = AL10(:,13);               %  12. | ghm     | GoHome                        | [U8]  | [12:12]   |
V10Log.PowerConsume.HoverDown = AL10(:,14);            %  13. | hvd     | HoverDown                     | [U8]  | [12:12]   |
V10Log.PowerConsume.Fix2Rotor = AL10(:,15);            %  14. | fix     | Fix2Rotor                     | [U8]  | [12:12]   |
V10Log.PowerConsume.Land = AL10(:,16);                 %  15. | lan     | Land                          | [U8]  | [12:12]   |
V10Log.PowerConsume.Sum = AL10(:,17);                  %  16. | Sm      | Sum                           | [U8]  | [12:12]   |
%% AL11
V10Log.OUT_TASKMODE.TimeUS = AL11(:,2);                %   1. | Tim     | TimeUS                        | [U64] | [12:12]   |
V10Log.OUT_TASKMODE.CNT = AL11(:,3);                   %   2. | CNT     | CNT                           | [U32] | [12:12]   |
V10Log.OUT_TASKMODE.currentPointNum = AL11(:,4);       %   3. | cup     | currentPointNum               | [U16] | [12:12]   |
V10Log.OUT_TASKMODE.prePointNum = AL11(:,5);           %   4. | prp     | prePointNum                   | [U16] | [12:12]   |
V10Log.OUT_TASKMODE.validPathNum = AL11(:,6);          %   5. | vap     | validPathNum                  | [U16] | [12:12]   |
V10Log.OUT_TASKMODE.headingCmd = AL11(:,7);            %   6. | hcd     | headingCmd                    | [F]   | [12:12]   |
V10Log.OUT_TASKMODE.distToGo = AL11(:,8);              %   7. | dtg     | distToGo                      | [F]   | [12:12]   |
V10Log.OUT_TASKMODE.dz = AL11(:,9);                    %   8. | dz      | dz                            | [F]   | [12:12]   |
V10Log.OUT_TASKMODE.groundspeedCmd = AL11(:,10);       %   9. | gsd     | groundspeedCmd                | [F]   | [12:12]   |
V10Log.OUT_TASKMODE.rollCmd = AL11(:,11);              %  10. | rcd     | rollCmd                       | [F]   | [12:12]   |
V10Log.OUT_TASKMODE.turnRadiusCmd = AL11(:,12);        %  11. | tcd     | turnRadiusCmd                 | [F]   | [12:12]   |
V10Log.OUT_TASKMODE.heightCmd = AL11(:,13);            %  12. | gcd     | heightCmd                     | [F]   | [12:12]   |
V10Log.OUT_TASKMODE.turnCenterLL_0 = AL11(:,14);       %  13. | LL0     | turnCenterLL_0                | [F]   | [12:12]   |
V10Log.OUT_TASKMODE.turnCenterLL_1 = AL11(:,15);       %  14. | LL1     | turnCenterLL_1                | [F]   | [12:12]   |
V10Log.OUT_TASKMODE.dR_turn = AL11(:,16);              %  15. | dR      | dR_turn                       | [F]   | [12:12]   |
V10Log.OUT_TASKMODE.Sum = AL11(:,17);                  %  16. | Sm      | Sum                           | [U8]  | [12:12]   |
%% AL12
V10Log.OUT_TASKMODE.TimeUS = AL12(:,2);                %   1. | Tim     | TimeUS                        | [U64] | [12:12]   |
V10Log.OUT_TASKMODE.CNT = AL12(:,3);                   %   2. | CNT     | CNT                           | [U32] | [12:12]   |
V10Log.OUT_TASKMODE.flightTaskMode = AL12(:,4);        %   3. | tMd     | flightTaskMode                | [U8]  | [12:12]   |
V10Log.OUT_TASKMODE.flightControlMode = AL12(:,5);     %   4. | cMd     | flightControlMode             | [U8]  | [12:12]   |
V10Log.OUT_TASKMODE.AutoManualMode = AL12(:,6);        %   5. | mMd     | AutoManualMode                | [U8]  | [12:12]   |
V10Log.OUT_TASKMODE.comStatus = AL12(:,7);             %   6. | com     | comStatus                     | [U8]  | [12:12]   |
V10Log.OUT_TASKMODE.maxClimbSpeed = AL12(:,8);         %   7. | mClp    | maxClimbSpeed                 | [F]   | [12:12]   |
V10Log.OUT_TASKMODE.prePathPoint_LLA_0 = AL12(:,9);    %   8. | LL0     | prePathPoint_LLA_0            | [F]   | [12:12]   |
V10Log.OUT_TASKMODE.prePathPoint_LLA_1 = AL12(:,10);   %   9. | LL1     | prePathPoint_LLA_1            | [F]   | [12:12]   |
V10Log.OUT_TASKMODE.prePathPoint_LLA_2 = AL12(:,11);   %  10. | LL2     | prePathPoint_LLA_2            | [F]   | [12:12]   |
V10Log.OUT_TASKMODE.curPathPoint_LLA_0 = AL12(:,12);   %  11. | LL3     | curPathPoint_LLA_0            | [F]   | [12:12]   |
V10Log.OUT_TASKMODE.curPathPoint_LLA_1 = AL12(:,13);   %  12. | LL4     | curPathPoint_LLA_1            | [F]   | [12:12]   |
V10Log.OUT_TASKMODE.curPathPoint_LLA_2 = AL12(:,14);   %  13. | LL5     | curPathPoint_LLA_2            | [F]   | [12:12]   |
V10Log.OUT_TASKMODE.whereIsUAV = AL12(:,15);           %  14. | UAV     | whereIsUAV                    | [F]   | [12:12]   |
V10Log.OUT_TASKMODE.typeAutoMode = AL12(:,16);         %  15. | aMd     | typeAutoMode                  | [U8]  | [12:12]   |
V10Log.OUT_TASKMODE.Sum = AL12(:,17);                  %  16. | Sm      | Sum                           | [U8]  | [12:12]   |
%% AL14
V10Log.OUT_TASKFLIGHTPARAM.TimeUS = AL14(:,2);         %   1. | Tim     | TimeUS                        | [U64] | [12:12]   |
V10Log.OUT_TASKFLIGHTPARAM.CNT = AL14(:,3);            %   2. | CNT     | CNT                           | [U32] | [12:12]   |
V10Log.OUT_TASKFLIGHTPARAM.curHomeLLA_0 = AL14(:,4);   %   3. | LL0     | curHomeLLA_0                  | [F]   | [12:12]   |
V10Log.OUT_TASKFLIGHTPARAM.curHomeLLA_1 = AL14(:,5);   %   4. | LL1     | curHomeLLA_1                  | [F]   | [12:12]   |
V10Log.OUT_TASKFLIGHTPARAM.curHomeLLA_2 = AL14(:,6);   %   5. | LL2     | curHomeLLA_2                  | [F]   | [12:12]   |
V10Log.OUT_TASKFLIGHTPARAM.curVelNED_0 = AL14(:,7);    %   6. | ND0     | curVelNED_0                   | [F]   | [12:12]   |
V10Log.OUT_TASKFLIGHTPARAM.curVelNED_1 = AL14(:,8);    %   7. | ND1     | curVelNED_1                   | [F]   | [12:12]   |
V10Log.OUT_TASKFLIGHTPARAM.curVelNED_2 = AL14(:,9);    %   8. | ND2     | curVelNED_2                   | [F]   | [12:12]   |
V10Log.OUT_TASKFLIGHTPARAM.curSpeed = AL14(:,10);      %   9. | aspd    | curSpeed                      | [F]   | [12:12]   |
V10Log.OUT_TASKFLIGHTPARAM.curAirSpeed = AL14(:,11);   %  10. | caspd   | curAirSpeed                   | [F]   | [12:12]   |
V10Log.OUT_TASKFLIGHTPARAM.curEuler_0 = AL14(:,12);    %  11. | Eul0    | curEuler_0                    | [F]   | [12:12]   |
V10Log.OUT_TASKFLIGHTPARAM.curEuler_1 = AL14(:,13);    %  12. | Eul1    | curEuler_1                    | [F]   | [12:12]   |
V10Log.OUT_TASKFLIGHTPARAM.curEuler_2 = AL14(:,14);    %  13. | Eul2    | curEuler_2                    | [F]   | [12:12]   |
V10Log.OUT_TASKFLIGHTPARAM.Sum = AL14(:,15);           %  14. | Sm      | Sum                           | [U8]  | [12:12]   |
%% AL15
V10Log.OUT_TASKFLIGHTPARAM.TimeUS = AL15(:,2);         %   1. | Tim     | TimeUS                        | [U64] | [12:12]   |
V10Log.OUT_TASKFLIGHTPARAM.CNT = AL15(:,3);            %   2. | CNT     | CNT                           | [U32] | [12:12]   |
V10Log.OUT_TASKFLIGHTPARAM.curWB_0 = AL15(:,4);        %   3. | WB0     | curWB_0                       | [F]   | [12:12]   |
V10Log.OUT_TASKFLIGHTPARAM.curWB_1 = AL15(:,5);        %   4. | WB1     | curWB_1                       | [F]   | [12:12]   |
V10Log.OUT_TASKFLIGHTPARAM.curWB_2 = AL15(:,6);        %   5. | WB2     | curWB_2                       | [F]   | [12:12]   |
V10Log.OUT_TASKFLIGHTPARAM.curPosNED_0 = AL15(:,7);    %   6. | ND0     | curPosNED_0                   | [F]   | [12:12]   |
V10Log.OUT_TASKFLIGHTPARAM.curPosNED_1 = AL15(:,8);    %   7. | ND1     | curPosNED_1                   | [F]   | [12:12]   |
V10Log.OUT_TASKFLIGHTPARAM.curPosNED_2 = AL15(:,9);    %   8. | ND2     | curPosNED_2                   | [F]   | [12:12]   |
V10Log.OUT_TASKFLIGHTPARAM.curLLA_0 = AL15(:,10);      %   9. | LL0     | curLLA_0                      | [F]   | [12:12]   |
V10Log.OUT_TASKFLIGHTPARAM.curLLA_0 = AL15(:,11);      %  10. | LL1     | curLLA_0                      | [F]   | [12:12]   |
V10Log.OUT_TASKFLIGHTPARAM.curLLA_0 = AL15(:,12);      %  11. | LL2     | curLLA_0                      | [F]   | [12:12]   |
V10Log.OUT_TASKFLIGHTPARAM.curGroundSpeed = AL15(:,13); %  12. | gspd    | curGroundSpeed                | [F]   | [12:12]   |
V10Log.OUT_TASKFLIGHTPARAM.curAccZ = AL15(:,14);       %  13. | cAcz    | curAccZ                       | [F]   | [12:12]   |
V10Log.OUT_TASKFLIGHTPARAM.Sum = AL15(:,15);           %  14. | Sm      | Sum                           | [U8]  | [12:12]   |
%% AL16
V10Log.OUT_TASKFLIGHTPARAM.TimeUS = AL16(:,2);         %   1. | Tim     | TimeUS                        | [U64] | [12:12]   |
V10Log.OUT_TASKFLIGHTPARAM.CNT = AL16(:,3);            %   2. | CNT     | CNT                           | [U32] | [12:12]   |
V10Log.OUT_TASKFLIGHTPARAM.groundHomeLLA_0 = AL16(:,4); %   3. | LL0     | groundHomeLLA_0               | [F]   | [12:12]   |
V10Log.OUT_TASKFLIGHTPARAM.groundHomeLLA_1 = AL16(:,5); %   4. | LL1     | groundHomeLLA_1               | [F]   | [12:12]   |
V10Log.OUT_TASKFLIGHTPARAM.groundHomeLLA_2 = AL16(:,6); %   5. | LL2     | groundHomeLLA_2               | [F]   | [12:12]   |
V10Log.OUT_TASKFLIGHTPARAM.curHeightForControl = AL16(:,7); %   6. | ctl     | curHeightForControl           | [F]   | [12:12]   |
V10Log.OUT_TASKFLIGHTPARAM.isNavFilterGood = AL16(:,8); %   7. | good    | isNavFilterGood               | [U8]  | [12:12]   |
V10Log.OUT_TASKFLIGHTPARAM.uavModel = AL16(:,9);       %   8. | mod     | uavModel                      | [U8]  | [12:12]   |
V10Log.OUT_TASKFLIGHTPARAM.Sum = AL16(:,10);           %   9. | Sm      | Sum                           | [U8]  | [12:12]   |
%% AL17
V10Log.OUT_FLIGHTPERF.TimeUS = AL17(:,2);              %   1. | Tim     | TimeUS                        | [U64] | [12:12]   |
V10Log.OUT_FLIGHTPERF.CNT = AL17(:,3);                 %   2. | CNT     | CNT                           | [U32] | [12:12]   |
V10Log.OUT_FLIGHTPERF.isAbleToCompleteTask = AL17(:,4); %   3. | cmp     | isAbleToCompleteTask          | [F]   | [12:12]   |
V10Log.OUT_FLIGHTPERF.flagGoHomeNow = AL17(:,5);       %   4. | ghm     | flagGoHomeNow                 | [F]   | [12:12]   |
V10Log.OUT_FLIGHTPERF.remainDistToGo_m = AL17(:,6);    %   5. | rdis    | remainDistToGo_m              | [F]   | [12:12]   |
V10Log.OUT_FLIGHTPERF.remainTimeToSpend_sec = AL17(:,7); %   6. | rtm     | remainTimeToSpend_sec         | [F]   | [12:12]   |
V10Log.OUT_FLIGHTPERF.remainPowerWhenFinish_per = AL17(:,8); %   7. | rpw     | remainPowerWhenFinish_per     | [F]   | [12:12]   |
V10Log.OUT_FLIGHTPERF.economicAirspeed = AL17(:,9);    %   8. | espd    | economicAirspeed              | [F]   | [12:12]   |
V10Log.OUT_FLIGHTPERF.remainPathPoint = AL17(:,10);    %   9. | rpnt    | remainPathPoint               | [F]   | [12:12]   |
V10Log.OUT_FLIGHTPERF.batteryLifeToCompleteTask = AL17(:,11); %  10. | bcmp    | batteryLifeToCompleteTask     | [F]   | [12:12]   |
V10Log.OUT_FLIGHTPERF.batterylifeNeededToHome = AL17(:,12); %  11. | bthm    | batterylifeNeededToHome       | [F]   | [12:12]   |
V10Log.OUT_FLIGHTPERF.isNavFilterGood = AL17(:,13);    %  12. | god     | isNavFilterGood               | [F]   | [12:12]   |
V10Log.OUT_FLIGHTPERF.groundHomeLLA = AL17(:,14);      %  13. | LLA     | groundHomeLLA                 | [F]   | [12:12]   |
V10Log.OUT_FLIGHTPERF.batterylifeNeededToLand = AL17(:,15); %  14. | btL     | batterylifeNeededToLand       | [F]   | [12:12]   |
V10Log.OUT_FLIGHTPERF.Sum = AL17(:,16);                %  15. | Sm      | Sum                           | [U8]  | [12:12]   |
%% AL18
V10Log.Debug_Task_RTInfo.TimeUS = AL18(:,2);           %   1. | Tim     | TimeUS                        | [U64] | [12:12]   |
V10Log.Debug_Task_RTInfo.CNT = AL18(:,3);              %   2. | CNT     | CNT                           | [U32] | [12:12]   |
V10Log.Debug_Task_RTInfo.Task = AL18(:,4);             %   3. | tsk     | Task                          | [F]   | [12:12]   |
V10Log.Debug_Task_RTInfo.Payload = AL18(:,5);          %   4. | pld     | Payload                       | [F]   | [12:12]   |
V10Log.Debug_Task_RTInfo.GSCmd = AL18(:,6);            %   5. | GSd     | GSCmd                         | [F]   | [12:12]   |
V10Log.Debug_Task_RTInfo.Warning = AL18(:,7);          %   6. | Wrn     | Warning                       | [F]   | [12:12]   |
V10Log.Debug_Task_RTInfo.ComStatus = AL18(:,8);        %   7. | Com     | ComStatus                     | [F]   | [12:12]   |
V10Log.Debug_Task_RTInfo.FenseStatus = AL18(:,9);      %   8. | Fen     | FenseStatus                   | [F]   | [12:12]   |
V10Log.Debug_Task_RTInfo.StallStatus = AL18(:,10);     %   9. | Stl     | StallStatus                   | [F]   | [12:12]   |
V10Log.Debug_Task_RTInfo.SensorStatus = AL18(:,11);    %  10. | Sen     | SensorStatus                  | [F]   | [12:12]   |
V10Log.Debug_Task_RTInfo.BatteryStatus = AL18(:,12);   %  11. | Bat     | BatteryStatus                 | [F]   | [12:12]   |
V10Log.Debug_Task_RTInfo.FixWingHeightStatus = AL18(:,13); %  12. | Fix     | FixWingHeightStatus           | [F]   | [12:12]   |
V10Log.Debug_Task_RTInfo.FindWind = AL18(:,14);        %  13. | Fin     | FindWind                      | [F]   | [12:12]   |
V10Log.Debug_Task_RTInfo.LandCond1_Acc_H = AL18(:,15); %  14. | LAc     | LandCond1_Acc_H               | [F]   | [12:12]   |
V10Log.Debug_Task_RTInfo.LandCond1_Vd_H = AL18(:,16);  %  15. | LVd     | LandCond1_Vd_H                | [F]   | [12:12]   |
V10Log.Debug_Task_RTInfo.Sum = AL18(:,17);             %  16. | Sm      | Sum                           | [U8]  | [12:12]   |
%% AL19
V10Log.Debug_Task_RTInfo.TimeUS = AL19(:,2);           %   1. | Tim     | TimeUS                        | [U64] | [12:12]   |
V10Log.Debug_Task_RTInfo.CNT = AL19(:,3);              %   2. | CNT     | CNT                           | [U32] | [12:12]   |
V10Log.Debug_Task_RTInfo.LandCond3_near = AL19(:,4);   %   3. | L3n     | LandCond3_near                | [F]   | [12:12]   |
V10Log.Debug_Task_RTInfo.maxDist_Path2Home = AL19(:,5); %   4. | hom     | maxDist_Path2Home             | [F]   | [12:12]   |
V10Log.Debug_Task_RTInfo.Sum = AL19(:,6);              %   5. | Sm      | Sum                           | [U8]  | [12:12]   |
%% AL20
V10Log.Debug_WindParam.TimeUS = AL20(:,2);             %   1. | Tim     | TimeUS                        | [U64] | [12:12]   |
V10Log.Debug_WindParam.CNT = AL20(:,3);                %   2. | CNT     | CNT                           | [U32] | [12:12]   |
V10Log.Debug_WindParam.sailWindSpeed = AL20(:,4);      %   3. | wspd    | sailWindSpeed                 | [F]   | [12:12]   |
V10Log.Debug_WindParam.sailWindHeading = AL20(:,5);    %   4. | swhd    | sailWindHeading               | [F]   | [12:12]   |
V10Log.Debug_WindParam.windSpeedMax = AL20(:,6);       %   5. | wmx     | windSpeedMax                  | [F]   | [12:12]   |
V10Log.Debug_WindParam.windSpeedMin = AL20(:,7);       %   6. | wmn     | windSpeedMin                  | [F]   | [12:12]   |
V10Log.Debug_WindParam.maxWindHeading = AL20(:,8);     %   7. | mwhd    | maxWindHeading                | [F]   | [12:12]   |
V10Log.Debug_WindParam.Sum = AL20(:,9);                %   8. | Sm      | Sum                           | [U8]  | [12:12]   |
%% AL21
V10Log.GlobalWindEst.TimeUS = AL21(:,2);               %   1. | Tim     | TimeUS                        | [U64] | [12:12]   |
V10Log.GlobalWindEst.CNT = AL21(:,3);                  %   2. | CNT     | CNT                           | [U32] | [12:12]   |
V10Log.GlobalWindEst.oneCircleComplete = AL21(:,4);    %   3. | comp    | oneCircleComplete             | [F]   | [12:12]   |
V10Log.GlobalWindEst.windSpeed_ms = AL21(:,5);         %   4. | wspd    | windSpeed_ms                  | [F]   | [12:12]   |
V10Log.GlobalWindEst.windHeading_rad = AL21(:,6);      %   5. | whrd    | windHeading_rad               | [F]   | [12:12]   |
V10Log.GlobalWindEst.Sum = AL21(:,7);                  %   6. | Sm      | Sum                           | [U8]  | [12:12]   |
%% AL23
V10Log.Debug_GroundStationShow.TimeUS = AL23(:,2);     %   1. | Tim     | TimeUS                        | [U64] | [12:12]   |
V10Log.Debug_GroundStationShow.CNT = AL23(:,3);        %   2. | CNT     | CNT                           | [U32] | [12:12]   |
V10Log.Debug_GroundStationShow.windSpeed_ms = AL23(:,4); %   3. | wspd    | windSpeed_ms                  | [F]   | [12:12]   |
V10Log.Debug_GroundStationShow.groundSpeed_ms = AL23(:,6); %   4. | gpsd    | groundSpeed_ms                | [F]   | [12:12]   |
%% AL24
V10Log.SimParam_LLA0.TimeUS = AL24(:,2);               %   1. | Tim     | TimeUS                        | [U64] | [12:12]   |
V10Log.SimParam_LLA0.CNT = AL24(:,3);                  %   2. | CNT     | CNT                           | [U32] | [12:12]   |
V10Log.SimParam_LLA0.SimParam_LLA0_0 = AL24(:,4);      %   3. | LL0     | SimParam_LLA0_0               | [F]   | [12:12]   |
V10Log.SimParam_LLA0.SimParam_LLA0_1 = AL24(:,5);      %   4. | LL1     | SimParam_LLA0_1               | [F]   | [12:12]   |
V10Log.SimParam_LLA0.SimParam_LLA0_2 = AL24(:,6);      %   5. | LL2     | SimParam_LLA0_2               | [F]   | [12:12]   |
V10Log.SimParam_LLA0.Sum = AL24(:,7);                  %   6. | Sm      | Sum                           | [U8]  | [12:12]   |
%% AL26
V10Log.OUT_NAVI2CONTROL.TimeUS = AL26(:,2);            %   1. | Tim     | TimeUS                        | [U64] | [12:12]   |
V10Log.OUT_NAVI2CONTROL.CNT = AL26(:,3);               %   2. | CNT     | CNT                           | [U32] | [12:12]   |
V10Log.OUT_NAVI2CONTROL.yawd = AL26(:,4);              %   3. | yad     | yawd                          | [F]   | [12:12]   |
V10Log.OUT_NAVI2CONTROL.pitchd = AL26(:,5);            %   4. | phd     | pitchd                        | [F]   | [12:12]   |
V10Log.OUT_NAVI2CONTROL.rolld = AL26(:,6);             %   5. | rod     | rolld                         | [F]   | [12:12]   |
V10Log.OUT_NAVI2CONTROL.latd = AL26(:,7);              %   6. | lad     | latd                          | [F]   | [12:12]   |
V10Log.OUT_NAVI2CONTROL.lond = AL26(:,8);              %   7. | lod     | lond                          | [F]   | [12:12]   |
V10Log.OUT_NAVI2CONTROL.alt = AL26(:,9);               %   8. | alt     | alt                           | [F]   | [12:12]   |
V10Log.OUT_NAVI2CONTROL.Vn = AL26(:,10);               %   9. | Vn      | Vn                            | [F]   | [12:12]   |
V10Log.OUT_NAVI2CONTROL.Ve = AL26(:,11);               %  10. | Ve      | Ve                            | [F]   | [12:12]   |
V10Log.OUT_NAVI2CONTROL.Vd = AL26(:,12);               %  11. | Vd      | Vd                            | [F]   | [12:12]   |
V10Log.OUT_NAVI2CONTROL.Sum = AL26(:,13);              %  12. | Sm      | Sum                           | [U8]  | [12:12]   |
%%
parserData = fieldnames(V10Log);
for i = 1:length(parserData)
    fprintf('output:		%s\n',parserData{i});
    assignin('base',parserData{i},V10Log.(parserData{i}));
end
parserData = fieldnames(V10Log.ALD1);
for i = 1:length(parserData)
    fprintf('output:		%s\n',parserData{i});
    assignin('base',parserData{i},V10Log.ALD1.(parserData{i}));
end
end
