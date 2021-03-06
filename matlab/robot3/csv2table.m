function datatestnoload2 = csv2table(filename, startRow, endRow)
%IMPORTFILE Import numeric data from a text file as a matrix.
%   DATATESTNOLOAD2 = IMPORTFILE(FILENAME) Reads data from text file
%   FILENAME for the default selection.
%
%   DATATESTNOLOAD2 = IMPORTFILE(FILENAME, STARTROW, ENDROW) Reads data
%   from rows STARTROW through ENDROW of text file FILENAME.
%
% Example:
%   datatestnoload2 = importfile('data_test_no_load_2.csv', 1, 4200);
%
%    See also TEXTSCAN.

% Auto-generated by MATLAB on 2019/02/24 11:07:13

%% Initialize variables.
delimiter = ' ';
if nargin<=2
    startRow = 1;
    endRow = inf;
end

%% Read columns of data as text:
% For more information, see the TEXTSCAN documentation.
formatSpec = '%s%s%s%s%s%s%s%s%s%s%s%s%s%s%[^\n\r]';

%% Open the text file.
fileID = fopen(filename,'r');

%% Read columns of data according to the format.
% This call is based on the structure of the file used to generate this
% code. If an error occurs for a different file, try regenerating the code
% from the Import Tool.
dataArray = textscan(fileID, formatSpec, endRow(1)-startRow(1)+1, 'Delimiter', delimiter, 'MultipleDelimsAsOne', true, 'TextType', 'string', 'HeaderLines', startRow(1)-1, 'ReturnOnError', false, 'EndOfLine', '\r\n');
for block=2:length(startRow)
    frewind(fileID);
    dataArrayBlock = textscan(fileID, formatSpec, endRow(block)-startRow(block)+1, 'Delimiter', delimiter, 'MultipleDelimsAsOne', true, 'TextType', 'string', 'HeaderLines', startRow(block)-1, 'ReturnOnError', false, 'EndOfLine', '\r\n');
    for col=1:length(dataArray)
        dataArray{col} = [dataArray{col};dataArrayBlock{col}];
    end
end

%% Close the text file.
fclose(fileID);

%% Convert the contents of columns containing numeric text to numbers.
% Replace non-numeric text with NaN.
raw = repmat({''},length(dataArray{1}),length(dataArray)-1);
for col=1:length(dataArray)-1
    raw(1:length(dataArray{col}),col) = mat2cell(dataArray{col}, ones(length(dataArray{col}), 1));
end
numericData = NaN(size(dataArray{1},1),size(dataArray,2));

for col=[2,4,6,8,9,10,11,12,14]
    % Converts text in the input cell array to numbers. Replaced non-numeric
    % text with NaN.
    rawData = dataArray{col};
    for row=1:size(rawData, 1)
        % Create a regular expression to detect and remove non-numeric prefixes and
        % suffixes.
        regexstr = '(?<prefix>.*?)(?<numbers>([-]*(\d+[\,]*)+[\.]{0,1}\d*[eEdD]{0,1}[-+]*\d*[i]{0,1})|([-]*(\d+[\,]*)*[\.]{1,1}\d+[eEdD]{0,1}[-+]*\d*[i]{0,1}))(?<suffix>.*)';
        try
            result = regexp(rawData(row), regexstr, 'names');
            numbers = result.numbers;
            
            % Detected commas in non-thousand locations.
            invalidThousandsSeparator = false;
            if numbers.contains(',')
                thousandsRegExp = '^\d+?(\,\d{3})*\.{0,1}\d*$';
                if isempty(regexp(numbers, thousandsRegExp, 'once'))
                    numbers = NaN;
                    invalidThousandsSeparator = true;
                end
            end
            % Convert numeric text to numbers.
            if ~invalidThousandsSeparator
                numbers = textscan(char(strrep(numbers, ',', '')), '%f');
                numericData(row, col) = numbers{1};
                raw{row, col} = numbers{1};
            end
        catch
            raw{row, col} = rawData{row};
        end
    end
end


%% Split data into numeric and string columns.
rawNumericColumns = raw(:, [2,4,6,8,9,10,11,12,14]);
rawStringColumns = string(raw(:, [1,3,5,7,13]));


%% Replace non-numeric cells with NaN
R = cellfun(@(x) ~isnumeric(x) && ~islogical(x),rawNumericColumns); % Find non-numeric cells
rawNumericColumns(R) = {NaN}; % Replace non-numeric cells

%% Make sure any text containing <undefined> is properly converted to an <undefined> categorical
for catIdx = [1,2,3,4,5]
    idx = (rawStringColumns(:, catIdx) == "<undefined>");
    rawStringColumns(idx, catIdx) = "";
end

%% Create output variable
datatestnoload2 = table;
datatestnoload2.W_left = categorical(rawStringColumns(:, 1));
datatestnoload2.VarName2 = cell2mat(rawNumericColumns(:, 1));
datatestnoload2.W_right = categorical(rawStringColumns(:, 2));
datatestnoload2.VarName4 = cell2mat(rawNumericColumns(:, 2));
datatestnoload2.sample_time = categorical(rawStringColumns(:, 3));
datatestnoload2.VarName6 = cell2mat(rawNumericColumns(:, 3));
datatestnoload2.time = categorical(rawStringColumns(:, 4));
datatestnoload2.VarName8 = cell2mat(rawNumericColumns(:, 4));
datatestnoload2.M2_analogleft = cell2mat(rawNumericColumns(:, 5));
datatestnoload2.VarName10 = cell2mat(rawNumericColumns(:, 6));
datatestnoload2.M1_analogright = cell2mat(rawNumericColumns(:, 7));
datatestnoload2.VarName12 = cell2mat(rawNumericColumns(:, 8));
datatestnoload2.PWM_value = categorical(rawStringColumns(:, 5));
datatestnoload2.VarName14 = cell2mat(rawNumericColumns(:, 9));

