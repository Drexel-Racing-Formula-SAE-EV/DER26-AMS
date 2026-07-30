function write_json(file_path, value)
%WRITE_JSON Write indented JSON with checked file handling.

file_path = char(file_path);
hil.ensure_directory(fileparts(file_path));

text = jsonencode(value, 'PrettyPrint', true);
fid = fopen(file_path, 'wt');
if fid < 0
    error('hil:io:OpenFailed', 'Could not open "%s" for writing.', file_path);
end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>

count = fprintf(fid, '%s\n', text);
if count <= 0
    error('hil:io:WriteFailed', 'Could not write JSON file "%s".', file_path);
end
end
