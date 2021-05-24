%V10 mat load

out_filePath=data_read_V10();
V10Log = V10_decode_auto(out_filePath);
time_algo=V10Log.BUS_CTRL.TimeUS*1e-6;
parserData = fieldnames(V10Log);
for i = 1:length(parserData)
    V10Log_parserData=V10Log.(parserData{i});
    V10Log_parserData_i=fieldnames(V10Log_parserData);
    V10Log_parserData=alignDimension(V10Log_parserData);
    if(strcmp(parserData{i,1},['BUS_CTRL','ALG2']))
        for j=1:length(V10Log_parserData_i)
            len=size(V10Log_parserData.(V10Log_parserData_i{j,1}),2);
            if(len<=1)
                assignin('base',['algo_',V10Log_parserData_i{j}],V10Log_parserData.(V10Log_parserData_i{j,1}))
            else
                for jj=1:len
                    assignin('base',['algo_',V10Log_parserData_i{j},'_',num2str(jj-1)],V10Log_parserData.(V10Log_parserData_i{j,1})(:,jj))
                end
            end
        end
    end 
    if(strcmp(parserData{i,1},'IMF1'))
        assignin('base',['time_imuf'],V10Log_parserData.(V10Log_parserData_i{1,1}))
        for j=2:length(V10Log_parserData_i)
            assignin('base',['algo_',V10Log_parserData_i{j},'_f'],V10Log_parserData.(V10Log_parserData_i{j,1}))
        end
    end
    if(strcmp(parserData{i,1},['BUS_CTRL','ALG2']))
        for j=1:length(V10Log_parserData_i)
            len=size(V10Log_parserData.(V10Log_parserData_i{j,1}),2);
            if(len<=1)
                assignin('base',['algo_',V10Log_parserData_i{j}],V10Log_parserData.(V10Log_parserData_i{j,1}))
            else
                for jj=1:len
                    assignin('base',['algo_',V10Log_parserData_i{j},'_',num2str(jj-1)],V10Log_parserData.(V10Log_parserData_i{j,1})(:,jj))
                end
            end
        end
    end
end



