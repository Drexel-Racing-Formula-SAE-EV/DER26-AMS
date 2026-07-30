function files = export_validation_report(report, output_directory)
%EXPORT_VALIDATION_REPORT Write machine-readable JSON and a concise Markdown report.

hil.ensure_directory(output_directory);
json_file = fullfile(output_directory, 'validation_report.json');
markdown_file = fullfile(output_directory, 'validation_report.md');

json_report = table_to_structs(report);
hil.write_json(json_file, json_report);

lines = {
    '# HIL validation report'
    ''
    sprintf('- Executed framework gates: **%s**', ...
        pass_text(report.framework_passed))
    sprintf('- Qualification complete: **%s**', ...
        yes_no(report.qualification_complete))
    sprintf('- Qualification result: **%s**', ...
        qualification_text(report))
    sprintf('- Cell configuration: `%s`', report.cell_configuration)
    sprintf('- Primary pack configuration: `%s`', report.pack_configuration)
    sprintf('- Parameter source: `%s`', report.parameter_source)
    sprintf('- Parameter hash: `%s`', report.parameter_hash)
    ''
    '## Qualification gate status'
    ''
    '| Gate | Status | Required here | Reason |'
    '|---|---:|---:|---|'
    };

gate_names = fieldnames(report.gates);
for index = 1:numel(gate_names)
    item = report.gates.(gate_names{index});
    label = strrep(gate_names{index}, '_', ' ');
    lines = [lines; {sprintf('| %s | **%s** | %s | %s |', ... %#ok<AGROW>
        label, item.status, yes_no(item.required), ...
        markdown_cell(item.reason))}];
end
lines = [lines; {
    ''
    '## Interpretation boundary'
    ''
    ['The checked-in generated plant retains one representative 2RC and ' ...
     'two-node thermal state. The MATLAB reference engine supports independent ' ...
     'per-group parameter and state evolution. Thermal results are comparative ' ...
     'screening, not fanless-pack qualification.']
    ''
    '## Deferred evidence'
    ''
    ['`PASS` means a gate executed and met its limits. `FAIL` means an ' ...
     'executed or required gate failed. `NOT_RUN` means no qualifying ' ...
     'evidence was produced in this run. P42A chamber temperature is ambient ' ...
     'evidence, not a measured cell-surface trace.']
    }];
write_text(markdown_file, strjoin(lines, newline));

files = struct('json', json_file, 'markdown', markdown_file);
end

function value = pass_text(passed)
if passed
    value = 'PASS';
else
    value = 'FAIL';
end
end

function value = yes_no(input)
if input
    value = 'YES';
else
    value = 'NO';
end
end

function value = qualification_text(report)
if ~report.qualification_complete
    value = 'NOT COMPLETE';
elseif report.qualification_passed
    value = 'PASS';
else
    value = 'FAIL';
end
end

function value = markdown_cell(input)
value = strrep(char(input), '|', '\|');
value = strrep(value, newline, ' ');
end

function output = table_to_structs(value)
if istable(value)
    output = table2struct(value);
elseif isstruct(value)
    output = value;
    names = fieldnames(value);
    for element = 1:numel(value)
        for index = 1:numel(names)
            output(element).(names{index}) = ...
                table_to_structs(value(element).(names{index}));
        end
    end
elseif iscell(value)
    output = cellfun(@table_to_structs, value, 'UniformOutput', false);
else
    output = value;
end
end

function write_text(file_path, content)
file = fopen(file_path, 'w');
if file < 0
    error('hil:file:WriteFailed', 'Could not write %s.', file_path);
end
cleanup = onCleanup(@() fclose(file)); %#ok<NASGU>
fprintf(file, '%s\n', content);
end
