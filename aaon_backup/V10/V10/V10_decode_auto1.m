function V10Log = V10_decode_auto(logFile)
% example: V10Log = V10_decode_auto('log_17.bin-2706669.mat')
% computer name: LAPTOP-KCGKQN65
% generate date: 11-Jan-2021
% Matlab version: 9.9.0.1467703 (R2020b)
% protocol file: V10_v20210107.txt
% data file: log_17.bin-2706669.mat
% logFile: .mat log file
load(logFile);
%% ARP1
% V10Log.ARP1.TimeUS = ARP1(:,2);                        %   1. | TimeUS  | TimeUS                         | [U64] | [10:10]   | 
% V10Log.ARP1.CNT = ARP1(:,3);                           %   2. | CNT     | CNT                            | [U32] | [10:10]   | 
% V10Log.ARP1.air_temp = ARP1(:,4);                      %   3. | tmp1    | air_temp                       | [F]   | [10:10]   | 
% V10Log.ARP1.air_diff_press_pa_raw = ARP1(:,5);         %   4. | prs1    | air_diff_press_pa_raw          | [F]   | [10:10]   | 
% V10Log.ARP1.indicated_airspeed = ARP1(:,6);            %   5. | isp1    | indicated_airspeed             | [F]   | [10:10]   | 
% V10Log.ARP1.true_airspeed = ARP1(:,7);                 %   6. | tsp1    | true_airspeed                  | [F]   | [10:10]   | 
% V10Log.ARP1.I2C_AirRetryCount_0(:,1) = ARP1(:,8);      %   7. | err1    | I2C_AirRetryCount[0][0]        | [U32] | [10:10]   | 
% V10Log.ARP1.I2C_AirRetryCount_0(:,2) = ARP1(:,9);      %   8. | err2    | I2C_AirRetryCount[0][1]        | [U32] | [10:10]   | 
% V10Log.ARP1.Sum = ARP1(:,10);                          %   9. | Sum     | Sum                            | [U8]  | [10:10]   | 
%% ARP2
V10Log.ARP2.TimeUS = ARP2(:,2);                        %   1. | TimeUS  | TimeUS                         | [U64] | [10:10]   | 
V10Log.ARP2.CNT = ARP2(:,3);                           %   2. | CNT     | CNT                            | [U32] | [10:10]   | 
V10Log.ARP2.air_temp = ARP2(:,4);                      %   3. | tmp2    | air_temp                       | [F]   | [10:10]   | 
V10Log.ARP2.air_diff_press_pa_raw = ARP2(:,5);         %   4. | prs2    | air_diff_press_pa_raw          | [F]   | [10:10]   | 
V10Log.ARP2.indicated_airspeed = ARP2(:,6);            %   5. | isp2    | indicated_airspeed             | [F]   | [10:10]   | 
V10Log.ARP2.true_airspeed = ARP2(:,7);                 %   6. | tsp2    | true_airspeed                  | [F]   | [10:10]   | 
V10Log.ARP2.I2C_AirRetryCount_1(:,1) = ARP2(:,8);      %   7. | err1    | I2C_AirRetryCount[1][0]        | [U32] | [10:10]   | 
V10Log.ARP2.I2C_AirRetryCount_1(:,2) = ARP2(:,9);      %   8. | err2    | I2C_AirRetryCount[1][1]        | [U32] | [10:10]   | 
V10Log.ARP2.Sum = ARP2(:,10);                          %   9. | Sum     | Sum                            | [U8]  | [10:10]   | 
%% MAG1
V10Log.MAG1.TimeUS = MAG1(:,2);                        %   1. | TimeUS  | TimeUS                         | [U64] | [10:10]   | 
V10Log.MAG1.CNT = MAG1(:,3);                           %   2. | CNT     | CNT                            | [U32] | [10:10]   | 
V10Log.MAG1.true_data_x = MAG1(:,4);                   %   3. | Lx_t    | true_data_x                    | [F]   | [10:10]   | 
V10Log.MAG1.true_data_y = MAG1(:,5);                   %   4. | Ly_t    | true_data_y                    | [F]   | [10:10]   | 
V10Log.MAG1.true_data_z = MAG1(:,6);                   %   5. | Lz_t    | true_data_z                    | [F]   | [10:10]   | 
V10Log.MAG1.cali_data_x = MAG1(:,7);                   %   6. | Lx_c    | cali_data_x                    | [F]   | [10:10]   | 
V10Log.MAG1.cali_data_y = MAG1(:,8);                   %   7. | Ly_c    | cali_data_y                    | [F]   | [10:10]   | 
V10Log.MAG1.cali_data_z = MAG1(:,9);                   %   8. | Lz_c    | cali_data_z                    | [F]   | [10:10]   | 
V10Log.MAG1.I2C_MagRetryCount_0(:,1) = MAG1(:,10);     %   9. | err1    | I2C_MagRetryCount[0][0]        | [U32] | [10:10]   | 
V10Log.MAG1.I2C_MagRetryCount_0(:,2) = MAG1(:,11);     %  10. | err2    | I2C_MagRetryCount[0][1]        | [U32] | [10:10]   | 
V10Log.MAG1.Sum = MAG1(:,12);                          %  11. | Sum     | Sum                            | [U8]  | [10:10]   | 
%% MAG2
V10Log.MAG2.TimeUS = MAG2(:,2);                        %   1. | TimeUS  | TimeUS                         | [U64] | [10:10]   | 
V10Log.MAG2.CNT = MAG2(:,3);                           %   2. | CNT     | CNT                            | [U32] | [10:10]   | 
V10Log.MAG2.true_data_x = MAG2(:,4);                   %   3. | Rx_t    | true_data_x                    | [F]   | [10:10]   | 
V10Log.MAG2.true_data_y = MAG2(:,5);                   %   4. | Ry_t    | true_data_y                    | [F]   | [10:10]   | 
V10Log.MAG2.true_data_z = MAG2(:,6);                   %   5. | Rz_t    | true_data_z                    | [F]   | [10:10]   | 
V10Log.MAG2.cali_data_x = MAG2(:,7);                   %   6. | Rx_c    | cali_data_x                    | [F]   | [10:10]   | 
V10Log.MAG2.cali_data_y = MAG2(:,8);                   %   7. | Ry_c    | cali_data_y                    | [F]   | [10:10]   | 
V10Log.MAG2.cali_data_z = MAG2(:,9);                   %   8. | Rz_c    | cali_data_z                    | [F]   | [10:10]   | 
V10Log.MAG2.I2C_MagRetryCount_1(:,1) = MAG2(:,10);     %   9. | err1    | I2C_MagRetryCount[1][0]        | [U32] | [10:10]   | 
V10Log.MAG2.I2C_MagRetryCount_1(:,2) = MAG2(:,11);     %  10. | err2    | I2C_MagRetryCount[1][1]        | [U32] | [10:10]   | 
V10Log.MAG2.Sum = MAG2(:,12);                          %  11. | Sum     | Sum                            | [U8]  | [10:10]   | 
%% BAR1
% V10Log.BAR1.TimeUS = BAR1(:,2);                        %   1. | TimeUS  | TimeUS                         | [U64] | [10:10]   | 
% V10Log.BAR1.CNT = BAR1(:,3);                           %   2. | CNT     | CNT                            | [U32] | [10:10]   | 
% V10Log.BAR1.TEMP = BAR1(:,4);                          %   3. | TEMP    | TEMP                           | [F]   | [10:10]   | 
% V10Log.BAR1.P = BAR1(:,5);                             %   4. | P       | P                              | [F]   | [10:10]   | 
% V10Log.BAR1.ground_pressure = BAR1(:,6);               %   5. | gP      | ground_pressure                | [F]   | [10:10]   | 
% V10Log.BAR1.ground_temperature = BAR1(:,7);            %   6. | gT      | ground_temperature             | [F]   | [10:10]   | 
% V10Log.BAR1.pressure = BAR1(:,8);                      %   7. | pres    | pressure                       | [F]   | [10:10]   | 
% V10Log.BAR1.altitude = BAR1(:,9);                      %   8. | alt     | altitude                       | [F]   | [10:10]   | 
% V10Log.BAR1.temperature = BAR1(:,10);                  %   9. | temp    | temperature                    | [F]   | [10:10]   | 
% V10Log.BAR1.Sum = BAR1(:,11);                          %  10. | Sum     | Sum                            | [U8]  | [10:10]   | 
% %% BAR2
% V10Log.BAR2.TimeUS = BAR2(:,2);                        %   1. | TimeUS  | TimeUS                         | [U64] | [10:10]   | 
% V10Log.BAR2.CNT = BAR2(:,3);                           %   2. | CNT     | CNT                            | [U32] | [10:10]   | 
% V10Log.BAR2.TEMP = BAR2(:,4);                          %   3. | TEMP    | TEMP                           | [F]   | [10:10]   | 
% V10Log.BAR2.P = BAR2(:,5);                             %   4. | P       | P                              | [F]   | [10:10]   | 
% V10Log.BAR2.ground_pressure = BAR2(:,6);               %   5. | gP      | ground_pressure                | [F]   | [10:10]   | 
% V10Log.BAR2.ground_temperature = BAR2(:,7);            %   6. | gT      | ground_temperature             | [F]   | [10:10]   | 
% V10Log.BAR2.pressure = BAR2(:,8);                      %   7. | pres    | pressure                       | [F]   | [10:10]   | 
% V10Log.BAR2.altitude = BAR2(:,9);                      %   8. | alt     | altitude                       | [F]   | [10:10]   | 
% V10Log.BAR2.temperature = BAR2(:,10);                  %   9. | temp    | temperature                    | [F]   | [10:10]   | 
% V10Log.BAR2.Sum = BAR2(:,11);                          %  10. | Sum     | Sum                            | [U8]  | [10:10]   | 
%% IMU1
V10Log.IMU1.TimeUS = IMU1(:,2);                        %   1. | TimeUS  | TimeUS                         | [U64] | [1:1]     | 
V10Log.IMU1.CNT = IMU1(:,3);                           %   2. | CNT     | CNT                            | [U32] | [1:1]     | 
V10Log.IMU1.ax = IMU1(:,4);                            %   3. | ax      | ax                             | [F]   | [1:1]     | 
V10Log.IMU1.ay = IMU1(:,5);                            %   4. | ay      | ay                             | [F]   | [1:1]     | 
V10Log.IMU1.az = IMU1(:,6);                            %   5. | az      | az                             | [F]   | [1:1]     | 
V10Log.IMU1.gx = IMU1(:,7);                            %   6. | gx      | gx                             | [F]   | [1:1]     | 
V10Log.IMU1.gy = IMU1(:,8);                            %   7. | gy      | gy                             | [F]   | [1:1]     | 
V10Log.IMU1.gz = IMU1(:,9);                            %   8. | gz      | gz                             | [F]   | [1:1]     | 
V10Log.IMU1.temperature = IMU1(:,10);                  %   9. | temp    | temperature                    | [F]   | [1:1]     | 
V10Log.IMU1.temp_pwm = IMU1(:,11);                     %  10. | pwm     | temp_pwm                       | [F]   | [1:1]     | 
V10Log.IMU1.Sum = IMU1(:,12);                          %  11. | Sum     | Sum                            | [U8]  | [1:1]     | 
%% IMU2
V10Log.IMU2.TimeUS = IMU2(:,2);                        %   1. | TimeUS  | TimeUS                         | [U64] | [1:1]     | 
V10Log.IMU2.CNT = IMU2(:,3);                           %   2. | CNT     | CNT                            | [U32] | [1:1]     | 
V10Log.IMU2.ax = IMU2(:,4);                            %   3. | ax      | ax                             | [F]   | [1:1]     | 
V10Log.IMU2.ay = IMU2(:,5);                            %   4. | ay      | ay                             | [F]   | [1:1]     | 
V10Log.IMU2.az = IMU2(:,6);                            %   5. | az      | az                             | [F]   | [1:1]     | 
V10Log.IMU2.gx = IMU2(:,7);                            %   6. | gx      | gx                             | [F]   | [1:1]     | 
V10Log.IMU2.gy = IMU2(:,8);                            %   7. | gy      | gy                             | [F]   | [1:1]     | 
V10Log.IMU2.gz = IMU2(:,9);                            %   8. | gz      | gz                             | [F]   | [1:1]     | 
V10Log.IMU2.temperature = IMU2(:,10);                  %   9. | temp    | temperature                    | [F]   | [1:1]     | 
V10Log.IMU2.temp_pwm = IMU2(:,11);                     %  10. | pwm     | temp_pwm                       | [F]   | [1:1]     | 
V10Log.IMU2.Sum = IMU2(:,12);                          %  11. | Sum     | Sum                            | [U8]  | [1:1]     | 
%% IMU3
V10Log.IMU3.TimeUS = IMU3(:,2);                        %   1. | TimeUS  | TimeUS                         | [U64] | [1:1]     | 
V10Log.IMU3.CNT = IMU3(:,3);                           %   2. | CNT     | CNT                            | [U32] | [1:1]     | 
V10Log.IMU3.ax = IMU3(:,4);                            %   3. | ax      | ax                             | [F]   | [1:1]     | 
V10Log.IMU3.ay = IMU3(:,5);                            %   4. | ay      | ay                             | [F]   | [1:1]     | 
V10Log.IMU3.az = IMU3(:,6);                            %   5. | az      | az                             | [F]   | [1:1]     | 
V10Log.IMU3.gx = IMU3(:,7);                            %   6. | gx      | gx                             | [F]   | [1:1]     | 
V10Log.IMU3.gy = IMU3(:,8);                            %   7. | gy      | gy                             | [F]   | [1:1]     | 
V10Log.IMU3.gz = IMU3(:,9);                            %   8. | gz      | gz                             | [F]   | [1:1]     | 
V10Log.IMU3.temperature = IMU3(:,10);                  %   9. | temp    | temperature                    | [F]   | [1:1]     | 
V10Log.IMU3.temp_pwm = IMU3(:,11);                     %  10. | pwm     | temp_pwm                       | [F]   | [1:1]     | 
V10Log.IMU3.Sum = IMU3(:,12);                          %  11. | Sum     | Sum                            | [U8]  | [1:1]     | 
%% IMU4
V10Log.IMU4.TimeUS = IMU4(:,2);                        %   1. | TimeUS  | TimeUS                         | [U64] | [1:1]     | 
V10Log.IMU4.CNT = IMU4(:,3);                           %   2. | CNT     | CNT                            | [U32] | [1:1]     | 
V10Log.IMU4.ax = IMU4(:,4);                            %   3. | ax      | ax                             | [F]   | [1:1]     | 
V10Log.IMU4.ay = IMU4(:,5);                            %   4. | ay      | ay                             | [F]   | [1:1]     | 
V10Log.IMU4.az = IMU4(:,6);                            %   5. | az      | az                             | [F]   | [1:1]     | 
V10Log.IMU4.gx = IMU4(:,7);                            %   6. | gx      | gx                             | [F]   | [1:1]     | 
V10Log.IMU4.gy = IMU4(:,8);                            %   7. | gy      | gy                             | [F]   | [1:1]     | 
V10Log.IMU4.gz = IMU4(:,9);                            %   8. | gz      | gz                             | [F]   | [1:1]     | 
V10Log.IMU4.temperature = IMU4(:,10);                  %   9. | temp    | temperature                    | [F]   | [1:1]     | 
V10Log.IMU4.temp_pwm = IMU4(:,11);                     %  10. | pwm     | temp_pwm                       | [F]   | [1:1]     | 
V10Log.IMU4.Sum = IMU4(:,12);                          %  11. | Sum     | Sum                            | [U8]  | [1:1]     | 
%% IMF1
V10Log.IMF1.TimeUS = IMF1(:,2);                        %   1. | TimeUS  | TimeUS                         | [U64] | [1:1]     | 
V10Log.IMF1.CNT = IMF1(:,3);                           %   2. | CNT     | CNT                            | [U32] | [1:1]     | 
V10Log.IMF1.ax = IMF1(:,4);                            %   3. | ax      | ax                             | [F]   | [1:1]     | 
V10Log.IMF1.ay = IMF1(:,5);                            %   4. | ay      | ay                             | [F]   | [1:1]     | 
V10Log.IMF1.az = IMF1(:,6);                            %   5. | az      | az                             | [F]   | [1:1]     | 
V10Log.IMF1.gx = IMF1(:,7);                            %   6. | gx      | gx                             | [F]   | [1:1]     | 
V10Log.IMF1.gy = IMF1(:,8);                            %   7. | gy      | gy                             | [F]   | [1:1]     | 
V10Log.IMF1.gz = IMF1(:,9);                            %   8. | gz      | gz                             | [F]   | [1:1]     | 
V10Log.IMF1.Sum = IMF1(:,10);                          %   9. | Sum     | Sum                            | [U8]  | [1:1]     | 
%% IMF5
V10Log.IMF5.TimeUS = IMF5(:,2);                        %   1. | TimeUS  | TimeUS                         | [U64] | [1:1]     | 
V10Log.IMF5.CNT = IMF5(:,3);                           %   2. | CNT     | CNT                            | [U32] | [1:1]     | 
V10Log.IMF5.ax = IMF5(:,4);                            %   3. | ax      | ax                             | [F]   | [1:1]     | 
V10Log.IMF5.ay = IMF5(:,5);                            %   4. | ay      | ay                             | [F]   | [1:1]     | 
V10Log.IMF5.az = IMF5(:,6);                            %   5. | az      | az                             | [F]   | [1:1]     | 
V10Log.IMF5.gx = IMF5(:,7);                            %   6. | gx      | gx                             | [F]   | [1:1]     | 
V10Log.IMF5.gy = IMF5(:,8);                            %   7. | gy      | gy                             | [F]   | [1:1]     | 
V10Log.IMF5.gz = IMF5(:,9);                            %   8. | gz      | gz                             | [F]   | [1:1]     | 
V10Log.IMF5.Sum = IMF5(:,10);                          %   9. | Sum     | Sum                            | [U8]  | [1:1]     | 
%% GPS
V10Log.GPS.TimeUS = GPS(:,2);                          %   1. | TimeUS  | TimeUS                         | [U64] | [50:50]   | 
V10Log.GPS.CNT = GPS(:,3);                             %   2. | CNT     | CNT                            | [U32] | [50:50]   | 
V10Log.GPS.lat = GPS(:,4);                             %   3. | lat     | lat                            | [D]   | [50:50]   | 
V10Log.GPS.lon = GPS(:,5);                             %   4. | lon     | lon                            | [D]   | [50:50]   | 
V10Log.GPS.height = GPS(:,6);                          %   5. | hgt     | height                         | [D]   | [50:50]   | 
V10Log.GPS.HDOP = GPS(:,7);                            %   6. | hdop    | HDOP                           | [F]   | [50:50]   | 
V10Log.GPS.PDOP = GPS(:,8);                            %   7. | pdop    | PDOP                           | [F]   | [50:50]   | 
V10Log.GPS.GPS_time_ms = GPS(:,9);                     %   8. | ms      | GPS_time_ms                    | [U32] | [50:50]   | 
V10Log.GPS.GPS_week_num = GPS(:,10);                   %   9. | week    | GPS_week_num                   | [U16] | [50:50]   | 
V10Log.GPS.solSVs = GPS(:,11);                         %  10. | solSVs  | solSVs                         | [U8]  | [50:50]   | 
V10Log.GPS.svn = GPS(:,12);                            %  11. | svn     | svn                            | [U8]  | [50:50]   | 
V10Log.GPS.Sum = GPS(:,13);                            %  12. | Sum     | Sum                            | [U8]  | [50:50]   | 
%% GPSE
V10Log.GPSE.TimeUS = GPSE(:,2);                        %   1. | TimeUS  | TimeUS                         | [U64] | [50:50]   | 
V10Log.GPSE.CNT = GPSE(:,3);                           %   2. | CNT     | CNT                            | [U32] | [50:50]   | 
V10Log.GPSE.decode_error_cnt = GPSE(:,4);              %   3. | err1    | decode_error_cnt               | [U32] | [50:50]   | 
V10Log.GPSE.decode_crc_error_cnt = GPSE(:,5);          %   4. | err2    | decode_crc_error_cnt           | [U32] | [50:50]   | 
V10Log.GPSE.decode_psrdop_cnt = GPSE(:,6);             %   5. | err3    | decode_psrdop_cnt              | [U32] | [50:50]   | 
V10Log.GPSE.oem718d_decode_status = GPSE(:,7);         %   6. | err4    | oem718d_decode_status          | [U32] | [50:50]   | 
V10Log.GPSE.decode_bestpos_cnt = GPSE(:,8);            %   7. | err5    | decode_bestpos_cnt             | [U32] | [50:50]   | 
V10Log.GPSE.decode_bestvel_cnt = GPSE(:,9);            %   8. | err6    | decode_bestvel_cnt             | [U32] | [50:50]   | 
V10Log.GPSE.decode_heading_cnt = GPSE(:,10);           %   9. | err7    | decode_heading_cnt             | [U32] | [50:50]   | 
V10Log.GPSE.rev = GPSE(:,11);                          %  10. | err8    | rev                            | [U32] | [50:50]   | 
V10Log.GPSE.Sum = GPSE(:,12);                          %  11. | Sum     | Sum                            | [U8]  | [50:50]   | 
%% UBX
V10Log.UBX.TimeUS = UBX(:,2);                          %   1. | TimeUS  | TimeUS                         | [U64] | [100:100] | 
V10Log.UBX.CNT = UBX(:,3);                             %   2. | CNT     | CNT                            | [U32] | [100:100] | 
V10Log.UBX.lat = UBX(:,4);                             %   3. | lat     | lat                            | [I32] | [100:100] | 
V10Log.UBX.lon = UBX(:,5);                             %   4. | lon     | lon                            | [I32] | [100:100] | 
V10Log.UBX.height = UBX(:,6);                          %   5. | hgt     | height                         | [I32] | [100:100] | 
V10Log.UBX.velN = UBX(:,7);                            %   6. | velN    | velN                           | [I32] | [100:100] | 
V10Log.UBX.velE = UBX(:,8);                            %   7. | velE    | velE                           | [I32] | [100:100] | 
V10Log.UBX.velD = UBX(:,9);                            %   8. | velD    | velD                           | [I32] | [100:100] | 
V10Log.UBX.hAcc = UBX(:,10);                           %   9. | hAcc    | hAcc                           | [U32] | [100:100] | 
V10Log.UBX.vAcc = UBX(:,11);                           %  10. | vAcc    | vAcc                           | [U32] | [100:100] | 
V10Log.UBX.pDOP = UBX(:,12);                           %  11. | pDOP    | pDOP                           | [U16] | [100:100] | 
V10Log.UBX.numSV = UBX(:,13);                          %  12. | numSV   | numSV                          | [U16] | [100:100] | 
V10Log.UBX.Sum = UBX(:,14);                            %  13. | Sum     | Sum                            | [U8]  | [100:100] | 
%% UBXE
V10Log.UBXE.TimeUS = UBXE(:,2);                        %   1. | TimeUS  | TimeUS                         | [U64] | [100:100] | 
V10Log.UBXE.CNT = UBXE(:,3);                           %   2. | CNT     | CNT                            | [U32] | [100:100] | 
V10Log.UBXE.decode_error_cnt = UBXE(:,4);              %   3. | err1    | decode_error_cnt               | [U32] | [100:100] | 
V10Log.UBXE.decode_crc_error_cnt = UBXE(:,4);          %   4. | err1    | decode_crc_error_cnt           | [U32] | [100:100] | 
V10Log.UBXE.decode_psrdop_cnt = UBXE(:,4);             %   5. | err1    | decode_psrdop_cnt              | [U32] | [100:100] | 
V10Log.UBXE.nak_error = UBXE(:,4);                     %   6. | err1    | nak_error                      | [U32] | [100:100] | 
V10Log.UBXE.rev1 = UBXE(:,4);                          %   7. | err1    | rev1                           | [U32] | [100:100] | 
V10Log.UBXE.rev2 = UBXE(:,4);                          %   8. | err1    | rev2                           | [U32] | [100:100] | 
V10Log.UBXE.rev3 = UBXE(:,4);                          %   9. | err1    | rev3                           | [U32] | [100:100] | 
V10Log.UBXE.rev4 = UBXE(:,4);                          %  10. | err1    | rev4                           | [U32] | [100:100] | 
V10Log.UBXE.Sum = UBXE(:,12);                          %  11. | Sum     | Sum                            | [U8]  | [100:100] | 
%% RCIN
V10Log.RCIN.TimeUS = RCIN(:,2);                        %   1. | TimeUS  | TimeUS                         | [U64] | [7:7]     | 
V10Log.RCIN.recv_total_cnt = RCIN(:,3);                %   2. | CNT     | recv_total_cnt                 | [U32] | [7:7]     | 
V10Log.RCIN.frame_lost_cnt = RCIN(:,4);                %   3. | LCNT    | frame_lost_cnt                 | [U32] | [7:7]     | 
V10Log.RCIN.channel_1_roll = RCIN(:,5);                %   4. | C1      | channel_1_roll                 | [U16] | [7:7]     | 
V10Log.RCIN.channel_2_pitch = RCIN(:,6);               %   5. | C2      | channel_2_pitch                | [U16] | [7:7]     | 
V10Log.RCIN.channel_3_throttle = RCIN(:,7);            %   6. | C3      | channel_3_throttle             | [U16] | [7:7]     | 
V10Log.RCIN.channel_4_yaw = RCIN(:,8);                 %   7. | C4      | channel_4_yaw                  | [U16] | [7:7]     | 
V10Log.RCIN.channel_5_Auto = RCIN(:,9);                %   8. | C5      | channel_5_Auto                 | [U16] | [7:7]     | 
V10Log.RCIN.channel_6_tilt = RCIN(:,10);               %   9. | C6      | channel_6_tilt                 | [U16] | [7:7]     | 
V10Log.RCIN.channel_7_D = RCIN(:,11);                  %  10. | C7      | channel_7_D                    | [U16] | [7:7]     | 
V10Log.RCIN.channel_8_C = RCIN(:,12);                  %  11. | C8      | channel_8_C                    | [U16] | [7:7]     | 
V10Log.RCIN.channel_9_Lock = RCIN(:,13);               %  12. | C9      | channel_9_Lock                 | [U16] | [7:7]     | 
V10Log.RCIN.channel_F = RCIN(:,14);                    %  13. | C10     | channel_F                      | [U16] | [7:7]     | 
V10Log.RCIN.Sum = RCIN(:,15);                          %  14. | Sum     | Sum                            | [U8]  | [7:7]     | 
%% PWMO
V10Log.PWMO.TimeUS = PWMO(:,2);                        %   1. | TimeUS  | TimeUS                         | [U64] | [12:12]   | 
V10Log.PWMO.count = PWMO(:,3);                         %   2. | CNT     | count                          | [U32] | [12:12]   | 
V10Log.PWMO.lost_cnt = PWMO(:,4);                      %   3. | LCNT    | lost_cnt                       | [U32] | [12:12]   | 
V10Log.PWMO.start_count = PWMO(:,5);                   %   4. | SCNT    | start_count                    | [U16] | [12:12]   | 
V10Log.PWMO.pwm_esc(:,1) = PWMO(:,6);                  %   5. | EC0     | pwm_esc[0]                     | [U16] | [12:12]   | 
V10Log.PWMO.pwm_esc(:,2) = PWMO(:,7);                  %   6. | EC1     | pwm_esc[1]                     | [U16] | [12:12]   | 
V10Log.PWMO.pwm_esc(:,3) = PWMO(:,8);                  %   7. | EC2     | pwm_esc[2]                     | [U16] | [12:12]   | 
V10Log.PWMO.pwm_esc(:,4) = PWMO(:,9);                  %   8. | EC3     | pwm_esc[3]                     | [U16] | [12:12]   | 
V10Log.PWMO.pwm_esc(:,5) = PWMO(:,10);                 %   9. | EC4     | pwm_esc[4]                     | [U16] | [12:12]   | 
V10Log.PWMO.pwm_servo(:,1) = PWMO(:,11);               %  10. | SV0     | pwm_servo[0]                   | [U16] | [12:12]   | 
V10Log.PWMO.pwm_servo(:,2) = PWMO(:,12);               %  11. | SV1     | pwm_servo[1]                   | [U16] | [12:12]   | 
V10Log.PWMO.pwm_servo(:,3) = PWMO(:,13);               %  12. | SV2     | pwm_servo[2]                   | [U16] | [12:12]   | 
V10Log.PWMO.pwm_servo(:,4) = PWMO(:,14);               %  13. | SV3     | pwm_servo[3]                   | [U16] | [12:12]   | 
V10Log.PWMO.pwm_servo(:,5) = PWMO(:,15);               %  14. | SV4     | pwm_servo[4]                   | [U16] | [12:12]   | 
V10Log.PWMO.Sum = PWMO(:,16);                          %  15. | Sum     | Sum                            | [U8]  | [12:12]   | 
%% ALD1
V10Log.CTRL.TimeUS = ALD1(:,2);                        %   1. | TimeUS  | TimeUS                         | [U64] | [12:12]   | 
V10Log.CTRL.CNT = ALD1(:,3);                           %   2. | CNT     | CNT                            | [U32] | [12:12]   | 
V10Log.CTRL.time = ALD1(:,4);                          %   3. | time    | time                           | [U64] | [12:12]   | 
V10Log.CTRL.mode = ALD1(:,5);                          %   4. | mode    | mode                           | [U8]  | [12:12]   | 
V10Log.CTRL.palne_mode = ALD1(:,6);                    %   5. | plmod   | palne_mode                     | [U8]  | [12:12]   | 
V10Log.CTRL.flightTaskMode = ALD1(:,7);                %   6. | fmod    | flightTaskMode                 | [U8]  | [12:12]   | 
V10Log.CTRL.PathMode = ALD1(:,8);                      %   7. | ptmod   | PathMode                       | [U8]  | [12:12]   | 
V10Log.CTRL.limit_pos_up = ALD1(:,9);                  %   8. | posup   | limit_pos_up                   | [U8]  | [12:12]   | 
V10Log.CTRL.throttle_upper = ALD1(:,10);               %   9. | thup    | throttle_upper                 | [U8]  | [12:12]   | 
V10Log.CTRL.throttle_lower = ALD1(:,11);               %  10. | thlo    | throttle_lower                 | [U8]  | [12:12]   | 
V10Log.CTRL.Sum = ALD1(:,12);                          %  11. | Sum     | Sum                            | [U8]  | [12:12]   | 
%% ALD2
V10Log.CTRL.TimeUS = ALD2(:,2);                        %   1. | Tim     | TimeUS                         | [U64] | [12:12]   | 
V10Log.CTRL.CNT = ALD2(:,3);                           %   2. | CNT     | CNT                            | [U32] | [12:12]   | 
V10Log.CTRL.roll = ALD2(:,4);                          %   3. | rol     | roll                           | [F]   | [12:12]   | 
V10Log.CTRL.pitch = ALD2(:,5);                         %   4. | pit     | pitch                          | [F]   | [12:12]   | 
V10Log.CTRL.yaw = ALD2(:,6);                           %   5. | yaw     | yaw                            | [F]   | [12:12]   | 
V10Log.CTRL.roll_in = ALD2(:,7);                       %   6. | rin     | roll_in                        | [F]   | [12:12]   | 
V10Log.CTRL.pitch_in = ALD2(:,8);                      %   7. | pin     | pitch_in                       | [F]   | [12:12]   | 
V10Log.CTRL.yaw_in = ALD2(:,9);                        %   8. | yin     | yaw_in                         | [F]   | [12:12]   | 
V10Log.CTRL.throttle_in = ALD2(:,10);                  %   9. | tin     | throttle_in                    | [F]   | [12:12]   | 
V10Log.CTRL.pwm_out(:,1) = ALD2(:,11);                 %  10. | pw0     | pwm_out[0]                     | [F]   | [12:12]   | 
V10Log.CTRL.pwm_out(:,2) = ALD2(:,12);                 %  11. | pw1     | pwm_out[1]                     | [F]   | [12:12]   | 
V10Log.CTRL.pwm_out(:,3) = ALD2(:,13);                 %  12. | pw2     | pwm_out[2]                     | [F]   | [12:12]   | 
V10Log.CTRL.pwm_out(:,4) = ALD2(:,14);                 %  13. | pw3     | pwm_out[3]                     | [F]   | [12:12]   | 
V10Log.CTRL.tail_tilt = ALD2(:,15);                    %  14. | til     | tail_tilt                      | [F]   | [12:12]   | 
V10Log.CTRL.pwm_tail = ALD2(:,16);                     %  15. | pwm     | pwm_tail                       | [F]   | [12:12]   | 
V10Log.CTRL.Sum = ALD2(:,17);                          %  16. | Sm      | Sum                            | [F]   | [12:12]   | 
%% ALD3
V10Log.CTRL.TimeUS = ALD3(:,2);                        %   1. | Tim     | TimeUS                         | [U64] | [12:12]   | 
V10Log.CTRL.CNT = ALD3(:,3);                           %   2. | CNT     | CNT                            | [U32] | [12:12]   | 
V10Log.CTRL.yaw_out = ALD3(:,4);                       %   3. | yout    | yaw_out                        | [F]   | [12:12]   | 
V10Log.CTRL.k_flap = ALD3(:,5);                        %   4. | flap    | k_flap                         | [F]   | [12:12]   | 
V10Log.CTRL.current_loc(:,1) = ALD3(:,6);              %   5. | cl0     | current_loc[0]                 | [F]   | [12:12]   | 
V10Log.CTRL.current_loc(:,2) = ALD3(:,7);              %   6. | cl1     | current_loc[1]                 | [F]   | [12:12]   | 
V10Log.CTRL.curr_vel(:,1) = ALD3(:,8);                 %   7. | cv0     | curr_vel[0]                    | [F]   | [12:12]   | 
V10Log.CTRL.curr_vel(:,2) = ALD3(:,9);                 %   8. | cv1     | curr_vel[1]                    | [F]   | [12:12]   | 
V10Log.CTRL.curr_vel(:,3) = ALD3(:,10);                %   9. | cv2     | curr_vel[2]                    | [F]   | [12:12]   | 
V10Log.CTRL.curr_pos(:,1) = ALD3(:,11);                %  10. | cp0     | curr_pos[0]                    | [F]   | [12:12]   | 
V10Log.CTRL.curr_pos(:,2) = ALD3(:,12);                %  11. | cp1     | curr_pos[1]                    | [F]   | [12:12]   | 
V10Log.CTRL.rate_target_ang_vel(:,1) = ALD3(:,13);     %  12. | rvl0    | rate_target_ang_vel[0]         | [F]   | [12:12]   | 
V10Log.CTRL.rate_target_ang_vel(:,2) = ALD3(:,14);     %  13. | rvl1    | rate_target_ang_vel[1]         | [F]   | [12:12]   | 
V10Log.CTRL.rate_target_ang_vel(:,3) = ALD3(:,15);     %  14. | rvl2    | rate_target_ang_vel[2]         | [F]   | [12:12]   | 
V10Log.CTRL.Sum = ALD3(:,16);                          %  15. | Sm      | Sum                            | [F]   | [12:12]   | 
%% ALD4
V10Log.CTRL.TimeUS = ALD4(:,2);                        %   1. | Tim     | TimeUS                         | [U64] | [12:12]   | 
V10Log.CTRL.CNT = ALD4(:,3);                           %   2. | CNT     | CNT                            | [U32] | [12:12]   | 
V10Log.CTRL.attitude_target_euler_rate(:,1) = ALD4(:,4); %   3. | er0     | attitude_target_euler_rate[0]  | [F]   | [12:12]   | 
V10Log.CTRL.attitude_target_euler_rate(:,2) = ALD4(:,5); %   4. | er1     | attitude_target_euler_rate[1]  | [F]   | [12:12]   | 
V10Log.CTRL.attitude_target_euler_rate(:,3) = ALD4(:,6); %   5. | er2     | attitude_target_euler_rate[2]  | [F]   | [12:12]   | 
V10Log.CTRL.attitude_target_euler_angle(:,1) = ALD4(:,7); %   6. | ea0     | attitude_target_euler_angle[0] | [F]   | [12:12]   | 
V10Log.CTRL.attitude_target_euler_angle(:,2) = ALD4(:,8); %   7. | ea1     | attitude_target_euler_angle[1] | [F]   | [12:12]   | 
V10Log.CTRL.attitude_target_euler_angle(:,3) = ALD4(:,9); %   8. | ea2     | attitude_target_euler_angle[2] | [F]   | [12:12]   | 
V10Log.CTRL.pos_target(:,1) = ALD4(:,10);              %   9. | ptg0    | pos_target[0]                  | [F]   | [12:12]   | 
V10Log.CTRL.pos_target(:,2) = ALD4(:,11);              %  10. | ptg1    | pos_target[1]                  | [F]   | [12:12]   | 
V10Log.CTRL.pos_target(:,3) = ALD4(:,12);              %  11. | ptg2    | pos_target[2]                  | [F]   | [12:12]   | 
V10Log.CTRL.vel_target(:,1) = ALD4(:,13);              %  12. | vtg0    | vel_target[0]                  | [F]   | [12:12]   | 
V10Log.CTRL.vel_target(:,2) = ALD4(:,14);              %  13. | vtg1    | vel_target[1]                  | [F]   | [12:12]   | 
V10Log.CTRL.vel_target(:,3) = ALD4(:,15);              %  14. | vtg2    | vel_target[2]                  | [F]   | [12:12]   | 
V10Log.CTRL.Sum = ALD4(:,16);                          %  15. | Sm      | Sum                            | [F]   | [12:12]   | 
%% ALD5
V10Log.CTRL.TimeUS = ALD5(:,2);                        %   1. | Tim     | TimeUS                         | [U64] | [12:12]   | 
V10Log.CTRL.CNT = ALD5(:,3);                           %   2. | CNT     | CNT                            | [U32] | [12:12]   | 
V10Log.CTRL.accel_target(:,1) = ALD5(:,4);             %   3. | acc0    | accel_target[0]                | [F]   | [12:12]   | 
V10Log.CTRL.accel_target(:,2) = ALD5(:,5);             %   4. | acc1    | accel_target[1]                | [F]   | [12:12]   | 
V10Log.CTRL.accel_target(:,3) = ALD5(:,6);             %   5. | acc2    | accel_target[2]                | [F]   | [12:12]   | 
V10Log.CTRL.attitude_error_vector(:,1) = ALD5(:,7);    %   6. | vect0   | attitude_error_vector[0]       | [F]   | [12:12]   | 
V10Log.CTRL.attitude_error_vector(:,2) = ALD5(:,8);    %   7. | vect1   | attitude_error_vector[1]       | [F]   | [12:12]   | 
V10Log.CTRL.attitude_error_vector(:,3) = ALD5(:,9);    %   8. | vect2   | attitude_error_vector[2]       | [F]   | [12:12]   | 
V10Log.CTRL.pos_error(:,1) = ALD5(:,10);               %   9. | per0    | pos_error[0]                   | [F]   | [12:12]   | 
V10Log.CTRL.pos_error(:,2) = ALD5(:,11);               %  10. | per1    | pos_error[1]                   | [F]   | [12:12]   | 
V10Log.CTRL.pos_error(:,3) = ALD5(:,12);               %  11. | per2    | pos_error[2]                   | [F]   | [12:12]   | 
V10Log.CTRL.vel_desired(:,3) = ALD5(:,13);             %  12. | veld2   | vel_desired[2]                 | [F]   | [12:12]   | 
V10Log.CTRL.Sum = ALD5(:,14);                          %  13. | Sm      | Sum                            | [F]   | [12:12]   | 
%% ALD6
V10Log.CTRL.TimeUS = ALD6(:,2);                        %   1. | Tim     | TimeUS                         | [U64] | [12:12]   | 
V10Log.CTRL.CNT = ALD6(:,3);                           %   2. | CNT     | CNT                            | [U32] | [12:12]   | 
V10Log.CTRL.z_accel_meas = ALD6(:,4);                  %   3. | zacm    | z_accel_meas                   | [F]   | [12:12]   | 
V10Log.CTRL.climb_rate_cms = ALD6(:,5);                %   4. | clrc    | climb_rate_cms                 | [F]   | [12:12]   | 
V10Log.CTRL.throttle_filter = ALD6(:,6);               %   5. | tfil    | throttle_filter                | [F]   | [12:12]   | 
V10Log.CTRL.nav_pitch_cd = ALD6(:,7);                  %   6. | navcd   | nav_pitch_cd                   | [F]   | [12:12]   | 
V10Log.CTRL.vel_forward_last_pct = ALD6(:,8);          %   7. | vlpct   | vel_forward_last_pct           | [F]   | [12:12]   | 
V10Log.CTRL.k_rudder = ALD6(:,9);                      %   8. | krud    | k_rudder                       | [F]   | [12:12]   | 
V10Log.CTRL.k_elevator = ALD6(:,10);                   %   9. | kele    | k_elevator                     | [F]   | [12:12]   | 
V10Log.CTRL.k_throttle = ALD6(:,11);                   %  10. | kthr    | k_throttle                     | [F]   | [12:12]   | 
V10Log.CTRL.k_aileron = ALD6(:,12);                    %  11. | kail    | k_aileron                      | [F]   | [12:12]   | 
V10Log.CTRL.curr_alt = ALD6(:,13);                     %  12. | curalt  | curr_alt                       | [F]   | [12:12]   | 
V10Log.CTRL.Sum = ALD6(:,14);                          %  13. | Sm      | Sum                            | [F]   | [12:12]   | 
%% ALD7
V10Log.CTRL.TimeUS = ALD7(:,2);                        %   1. | Tim     | TimeUS                         | [U64] | [12:12]   | 
V10Log.CTRL.CNT = ALD7(:,3);                           %   2. | CNT     | CNT                            | [U32] | [12:12]   | 
V10Log.CTRL.weathervane_last_output = ALD7(:,4);       %   3. | weat    | weathervane_last_output        | [F]   | [12:12]   | 
V10Log.CTRL.roll_target = ALD7(:,5);                   %   4. | rotg    | roll_target                    | [F]   | [12:12]   | 
V10Log.CTRL.pitch_target = ALD7(:,6);                  %   5. | pitg    | pitch_target                   | [F]   | [12:12]   | 
V10Log.CTRL.roll_target_pilot = ALD7(:,7);             %   6. | rotp    | roll_target_pilot              | [F]   | [12:12]   | 
V10Log.CTRL.pitch_dem = ALD7(:,8);                     %   7. | pitdm   | pitch_dem                      | [F]   | [12:12]   | 
V10Log.CTRL.hgt_dem = ALD7(:,9);                       %   8. | hgtdm   | hgt_dem                        | [F]   | [12:12]   | 
V10Log.CTRL.throttle_dem = ALD7(:,10);                 %   9. | thdm    | throttle_dem                   | [F]   | [12:12]   | 
V10Log.CTRL.latAccDem = ALD7(:,11);                    %  10. | accdm   | latAccDem                      | [F]   | [12:12]   | 
V10Log.CTRL.aspeed = ALD7(:,12);                       %  11. | aspd    | aspeed                         | [F]   | [12:12]   | 
V10Log.CTRL.pitch_target_pilot = ALD7(:,13);           %  12. | pitpi   | pitch_target_pilot             | [F]   | [12:12]   | 
V10Log.CTRL.Sum = ALD7(:,14);                          %  13. | Sm      | Sum                            | [F]   | [12:12]   | 
%% ALD8
V10Log.CTRL.TimeUS = ALD8(:,2);                        %   1. | Tim     | TimeUS                         | [U64] | [12:12]   | 
V10Log.CTRL.CNT = ALD8(:,3);                           %   2. | CNT     | CNT                            | [U32] | [12:12]   | 
V10Log.CTRL.WP_i = ALD8(:,4);                          %   3. | WP_i    | WP_i                           | [F]   | [12:12]   | 
V10Log.CTRL.sl_heightCmd = ALD8(:,5);                  %   4. | hgtcmd  | sl_heightCmd                   | [F]   | [12:12]   | 
V10Log.CTRL.sl_maxClimbSpeed = ALD8(:,6);              %   5. | clmspd  | sl_maxClimbSpeed               | [F]   | [12:12]   | 
V10Log.CTRL.sl_flightTaskMode = ALD8(:,7);             %   6. | fmode   | sl_flightTaskMode              | [F]   | [12:12]   | 
V10Log.CTRL.Sum = ALD8(:,8);                           %   7. | Sm      | Sum                            | [F]   | [12:12]   | 
%% ALL4
V10Log.SensorSelect.TimeUS = ALL4(:,2);                %   1. | Tim     | TimeUS                         | [U64] | [12:12]   | 
V10Log.SensorSelect.CNT = ALL4(:,3);                   %   2. | CNT     | CNT                            | [U32] | [12:12]   | 
V10Log.SensorSelect.IMU = ALL4(:,4);                   %   3. | IMU     | IMU                            | [F]   | [12:12]   | 
V10Log.SensorSelect.Mag = ALL4(:,5);                   %   4. | Mag     | Mag                            | [F]   | [12:12]   | 
V10Log.SensorSelect.GPS = ALL4(:,6);                   %   5. | GPS     | GPS                            | [F]   | [12:12]   | 
V10Log.SensorSelect.Baro = ALL4(:,7);                  %   6. | Bar     | Baro                           | [F]   | [12:12]   | 
V10Log.SensorSelect.Radar = ALL4(:,8);                 %   7. | Radr    | Radar                          | [F]   | [12:12]   | 
V10Log.SensorSelect.Camera = ALL4(:,9);                %   8. | CAM     | Camera                         | [F]   | [12:12]   | 
V10Log.SensorSelect.Lidar = ALL4(:,10);                %   9. | Lidr    | Lidar                          | [F]   | [12:12]   | 
%% ALL5
V10Log.SensorUpdateFlag.TimeUS = ALL5(:,2);            %   1. | Tim     | TimeUS                         | [U64] | [12:12]   | 
V10Log.SensorUpdateFlag.CNT = ALL5(:,3);               %   2. | CNT     | CNT                            | [U32] | [12:12]   | 
V10Log.SensorUpdateFlag.mag1 = ALL5(:,4);              %   3. | mg1     | mag1                           | [U8]  | [12:12]   | 
V10Log.SensorUpdateFlag.mag2 = ALL5(:,5);              %   4. | mg2     | mag2                           | [U8]  | [12:12]   | 
V10Log.SensorUpdateFlag.airspeed1 = ALL5(:,6);         %   5. | ap1     | airspeed1                      | [U8]  | [12:12]   | 
V10Log.SensorUpdateFlag.airspeed2 = ALL5(:,7);         %   6. | ap2     | airspeed2                      | [U8]  | [12:12]   | 
V10Log.SensorUpdateFlag.baro1 = ALL5(:,8);             %   7. | br1     | baro1                          | [U8]  | [12:12]   | 
V10Log.SensorUpdateFlag.baro2 = ALL5(:,9);             %   8. | br2     | baro2                          | [U8]  | [12:12]   | 
V10Log.SensorUpdateFlag.IMU1 = ALL5(:,10);             %   9. | m1      | IMU1                           | [U8]  | [12:12]   | 
V10Log.SensorUpdateFlag.IMU2 = ALL5(:,11);             %  10. | m2      | IMU2                           | [U8]  | [12:12]   | 
V10Log.SensorUpdateFlag.IMU3 = ALL5(:,12);             %  11. | m3      | IMU3                           | [U8]  | [12:12]   | 
V10Log.SensorUpdateFlag.IMU4 = ALL5(:,13);             %  12. | m4      | IMU4                           | [U8]  | [12:12]   | 
V10Log.SensorUpdateFlag.um482 = ALL5(:,14);            %  13. | um482   | um482                          | [U8]  | [12:12]   | 
V10Log.SensorUpdateFlag.ublox1 = ALL5(:,15);           %  14. | ubx1    | ublox1                         | [U8]  | [12:12]   | 
V10Log.SensorUpdateFlag.radar1 = ALL5(:,16);           %  15. | radr1   | radar1                         | [U8]  | [12:12]   | 
V10Log.SensorUpdateFlag.Sum = ALL5(:,17);              %  16. | Sm      | Sum                            | [U8]  | [12:12]   | 
%% ALL6
V10Log.SensorLosttime.TimeUS = ALL6(:,2);              %   1. | Tim     | TimeUS                         | [U64] | [12:12]   | 
V10Log.SensorLosttime.CNT = ALL6(:,3);                 %   2. | CNT     | CNT                            | [U32] | [12:12]   | 
V10Log.SensorLosttime.mag1 = ALL6(:,4);                %   3. | mg1     | mag1                           | [F]   | [12:12]   | 
V10Log.SensorLosttime.mag2 = ALL6(:,5);                %   4. | mg2     | mag2                           | [F]   | [12:12]   | 
V10Log.SensorLosttime.airspeed1 = ALL6(:,6);           %   5. | ap1     | airspeed1                      | [F]   | [12:12]   | 
V10Log.SensorLosttime.airspeed2 = ALL6(:,7);           %   6. | ap2     | airspeed2                      | [F]   | [12:12]   | 
V10Log.SensorLosttime.baro1 = ALL6(:,8);               %   7. | br1     | baro1                          | [F]   | [12:12]   | 
V10Log.SensorLosttime.baro2 = ALL6(:,9);               %   8. | br2     | baro2                          | [F]   | [12:12]   | 
V10Log.SensorLosttime.IMU1 = ALL6(:,10);               %   9. | m1      | IMU1                           | [F]   | [12:12]   | 
V10Log.SensorLosttime.IMU2 = ALL6(:,11);               %  10. | m2      | IMU2                           | [F]   | [12:12]   | 
V10Log.SensorLosttime.IMU3 = ALL6(:,12);               %  11. | m3      | IMU3                           | [F]   | [12:12]   | 
V10Log.SensorLosttime.IMU4 = ALL6(:,13);               %  12. | m4      | IMU4                           | [F]   | [12:12]   | 
V10Log.SensorLosttime.um482 = ALL6(:,14);              %  13. | um482   | um482                          | [F]   | [12:12]   | 
V10Log.SensorLosttime.ublox1 = ALL6(:,15);             %  14. | ubx1    | ublox1                         | [F]   | [12:12]   | 
V10Log.SensorLosttime.radar1 = ALL6(:,16);             %  15. | radr1   | radar1                         | [F]   | [12:12]   | 
V10Log.SensorLosttime.Sum = ALL6(:,17);                %  16. | Sm      | Sum                            | [U8]  | [12:12]   | 
%% AL12
V10Log.OUT_TASKMODE.TimeUS = AL12(:,2);                %   1. | Tim     | TimeUS                         | [U64] | [12:12]   | 
V10Log.OUT_TASKMODE.CNT = AL12(:,3);                   %   2. | CNT     | CNT                            | [U32] | [12:12]   | 
V10Log.OUT_TASKMODE.flightTaskMode = AL12(:,4);        %   3. | tMd     | flightTaskMode                 | [U8]  | [12:12]   | 
V10Log.OUT_TASKMODE.flightControlMode = AL12(:,5);     %   4. | cMd     | flightControlMode              | [U8]  | [12:12]   | 
V10Log.OUT_TASKMODE.AutoManualMode = AL12(:,6);        %   5. | mMd     | AutoManualMode                 | [U8]  | [12:12]   | 
V10Log.OUT_TASKMODE.comStatus = AL12(:,7);             %   6. | com     | comStatus                      | [U8]  | [12:12]   | 
V10Log.OUT_TASKMODE.maxClimbSpeed = AL12(:,8);         %   7. | mClp    | maxClimbSpeed                  | [F]   | [12:12]   | 
V10Log.OUT_TASKMODE.prePathPoint_LLA(:,1) = AL12(:,9); %   8. | LL0     | prePathPoint_LLA[0]            | [F]   | [12:12]   | 
V10Log.OUT_TASKMODE.prePathPoint_LLA(:,2) = AL12(:,10); %   9. | LL1     | prePathPoint_LLA[1]            | [F]   | [12:12]   | 
V10Log.OUT_TASKMODE.prePathPoint_LLA(:,3) = AL12(:,11); %  10. | LL2     | prePathPoint_LLA[2]            | [F]   | [12:12]   | 
V10Log.OUT_TASKMODE.curPathPoint_LLA(:,1) = AL12(:,12); %  11. | LL3     | curPathPoint_LLA[0]            | [F]   | [12:12]   | 
V10Log.OUT_TASKMODE.curPathPoint_LLA(:,2) = AL12(:,13); %  12. | LL4     | curPathPoint_LLA[1]            | [F]   | [12:12]   | 
V10Log.OUT_TASKMODE.curPathPoint_LLA(:,3) = AL12(:,14); %  13. | LL5     | curPathPoint_LLA[2]            | [F]   | [12:12]   | 
V10Log.OUT_TASKMODE.whereIsUAV = AL12(:,15);           %  14. | UAV     | whereIsUAV                     | [F]   | [12:12]   | 
V10Log.OUT_TASKMODE.typeAutoMode = AL12(:,16);         %  15. | aMd     | typeAutoMode                   | [U8]  | [12:12]   | 
V10Log.OUT_TASKMODE.Sum = AL12(:,17);                  %  16. | Sm      | Sum                            | [U8]  | [12:12]   | 
%% AL14
V10Log.OUT_TASKFLIGHTPARAM.TimeUS = AL14(:,2);         %   1. | Tim     | TimeUS                         | [U64] | [12:12]   | 
V10Log.OUT_TASKFLIGHTPARAM.CNT = AL14(:,3);            %   2. | CNT     | CNT                            | [U32] | [12:12]   | 
V10Log.OUT_TASKFLIGHTPARAM.curHomeLLA(:,1) = AL14(:,4); %   3. | LL0     | curHomeLLA[0]                  | [F]   | [12:12]   | 
V10Log.OUT_TASKFLIGHTPARAM.curHomeLLA(:,2) = AL14(:,5); %   4. | LL1     | curHomeLLA[1]                  | [F]   | [12:12]   | 
V10Log.OUT_TASKFLIGHTPARAM.curHomeLLA(:,3) = AL14(:,6); %   5. | LL2     | curHomeLLA[2]                  | [F]   | [12:12]   | 
V10Log.OUT_TASKFLIGHTPARAM.curVelNED(:,1) = AL14(:,7); %   6. | ND0     | curVelNED[0]                   | [F]   | [12:12]   | 
V10Log.OUT_TASKFLIGHTPARAM.curVelNED(:,2) = AL14(:,8); %   7. | ND1     | curVelNED[1]                   | [F]   | [12:12]   | 
V10Log.OUT_TASKFLIGHTPARAM.curVelNED(:,3) = AL14(:,9); %   8. | ND2     | curVelNED[2]                   | [F]   | [12:12]   | 
V10Log.OUT_TASKFLIGHTPARAM.curSpeed = AL14(:,11);      %   9. | aspd    | curSpeed                       | [F]   | [12:12]   | 
V10Log.OUT_TASKFLIGHTPARAM.curAirSpeed = AL14(:,12);   %  10. | caspd   | curAirSpeed                    | [F]   | [12:12]   | 
V10Log.OUT_TASKFLIGHTPARAM.curEuler(:,1) = AL14(:,13); %  11. | Eul0    | curEuler[0]                    | [F]   | [12:12]   | 
V10Log.OUT_TASKFLIGHTPARAM.curEuler(:,2) = AL14(:,14); %  12. | Eul1    | curEuler[1]                    | [F]   | [12:12]   | 
V10Log.OUT_TASKFLIGHTPARAM.curEuler(:,3) = AL14(:,15); %  13. | Eul2    | curEuler[2]                    | [F]   | [12:12]   | 
%% AL15
V10Log.OUT_TASKFLIGHTPARAM.TimeUS = AL15(:,2);         %   1. | Tim     | TimeUS                         | [U64] | [12:12]   | 
V10Log.OUT_TASKFLIGHTPARAM.CNT = AL15(:,3);            %   2. | CNT     | CNT                            | [U32] | [12:12]   | 
V10Log.OUT_TASKFLIGHTPARAM.curWB(:,1) = AL15(:,4);     %   3. | WB0     | curWB[0]                       | [F]   | [12:12]   | 
V10Log.OUT_TASKFLIGHTPARAM.curWB(:,2) = AL15(:,5);     %   4. | WB1     | curWB[1]                       | [F]   | [12:12]   | 
V10Log.OUT_TASKFLIGHTPARAM.curWB(:,3) = AL15(:,6);     %   5. | WB2     | curWB[2]                       | [F]   | [12:12]   | 
V10Log.OUT_TASKFLIGHTPARAM.curPosNED(:,1) = AL15(:,7); %   6. | ND0     | curPosNED[0]                   | [F]   | [12:12]   | 
V10Log.OUT_TASKFLIGHTPARAM.curPosNED(:,2) = AL15(:,8); %   7. | ND1     | curPosNED[1]                   | [F]   | [12:12]   | 
V10Log.OUT_TASKFLIGHTPARAM.curPosNED(:,3) = AL15(:,9); %   8. | ND2     | curPosNED[2]                   | [F]   | [12:12]   | 
V10Log.OUT_TASKFLIGHTPARAM.curLLA(:,1) = AL15(:,10);   %   9. | LL0     | curLLA[0]                      | [F]   | [12:12]   | 
V10Log.OUT_TASKFLIGHTPARAM.curLLA(:,1) = AL15(:,11);   %  10. | LL1     | curLLA[0]                      | [F]   | [12:12]   | 
V10Log.OUT_TASKFLIGHTPARAM.curLLA(:,1) = AL15(:,12);   %  11. | LL2     | curLLA[0]                      | [F]   | [12:12]   | 
V10Log.OUT_TASKFLIGHTPARAM.curGroundSpeed = AL15(:,13); %  12. | gspd    | curGroundSpeed                 | [F]   | [12:12]   | 
V10Log.OUT_TASKFLIGHTPARAM.curAccZ = AL15(:,14);       %  13. | cAcz    | curAccZ                        | [F]   | [12:12]   | 
V10Log.OUT_TASKFLIGHTPARAM.Sum = AL15(:,15);           %  14. | Sm      | Sum                            | [U8]  | [12:12]   | 
%% AL26
V10Log.OUT_NAVI2CONTROL.TimeUS = AL26(:,2);            %   1. | Tim     | TimeUS                         | [U64] | [12:12]   | 
V10Log.OUT_NAVI2CONTROL.CNT = AL26(:,3);               %   2. | CNT     | CNT                            | [U32] | [12:12]   | 
V10Log.OUT_NAVI2CONTROL.yawd = AL26(:,4);              %   3. | yad     | yawd                           | [F]   | [12:12]   | 
V10Log.OUT_NAVI2CONTROL.pitchd = AL26(:,5);            %   4. | phd     | pitchd                         | [F]   | [12:12]   | 
V10Log.OUT_NAVI2CONTROL.rolld = AL26(:,6);             %   5. | rod     | rolld                          | [F]   | [12:12]   | 
V10Log.OUT_NAVI2CONTROL.latd = AL26(:,7);              %   6. | lad     | latd                           | [F]   | [12:12]   | 
V10Log.OUT_NAVI2CONTROL.lond = AL26(:,8);              %   7. | lod     | lond                           | [F]   | [12:12]   | 
V10Log.OUT_NAVI2CONTROL.alt = AL26(:,9);               %   8. | alt     | alt                            | [F]   | [12:12]   | 
V10Log.OUT_NAVI2CONTROL.Vn = AL26(:,10);               %   9. | Vn      | Vn                             | [F]   | [12:12]   | 
V10Log.OUT_NAVI2CONTROL.Ve = AL26(:,11);               %  10. | Ve      | Ve                             | [F]   | [12:12]   | 
V10Log.OUT_NAVI2CONTROL.Vd = AL26(:,12);               %  11. | Vd      | Vd                             | [F]   | [12:12]   | 
V10Log.OUT_NAVI2CONTROL.Sum = AL26(:,13);              %  12. | Sm      | Sum                            | [U8]  | [12:12]   | 
%% 
parserData = fieldnames(V10Log);
for i = 1:length(parserData)
	fprintf('output:		%s\n',parserData{i});
	assignin('base',parserData{i},V10Log.(parserData{i}));
end
