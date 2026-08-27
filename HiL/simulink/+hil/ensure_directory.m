function path_out = ensure_directory(path_in)
%ENSURE_DIRECTORY Create a directory when it does not already exist.

path_out = char(path_in);
if isempty(path_out)
    error('hil:path:EmptyDirectory', 'Directory path must not be empty.');
end

if exist(path_out, 'dir') ~= 7
    [ok, message] = mkdir(path_out);
    if ~ok
        error('hil:path:CreateFailed', ...
            'Could not create directory "%s": %s', path_out, message);
    end
end
end
