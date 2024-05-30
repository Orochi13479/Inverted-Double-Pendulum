% Create CSV file of Trajectory Generation Data
% Define the filename
filename = 'trajectory_data_1.csv';

% Load data into a table for processing
data_table = readtable(filename);

% Write the table to an Excel file
output_excel = 'highlighted_trajectory_data.xlsx';
writetable(data_table, output_excel, 'Sheet', 1);

% Specify the range to highlight
lower_bound = -1.7;
upper_bound = 1.7;

% Open the Excel file for editing
excel = actxserver('Excel.Application');
workbook = excel.Workbooks.Open(fullfile(pwd, output_excel));
sheet = workbook.Sheets.Item(1);

% Get the range of cells to check (excluding header)
range = sheet.Range('B2:I1000'); % Adjust range as necessary

% Loop through each cell in the range
for r = 1:size(range, 1)
    for c = 1:size(range, 2)
        cell = range.Item(r, c);
        value = cell.Value;
        if value < lower_bound || value > upper_bound
            cell.Interior.Color = 65535; % Yellow color
        end
    end
end

% Save and close the workbook
workbook.Save();
workbook.Close(false);
excel.Quit();

% Clean up
delete(excel);

