function values = parse_int16_c_array(file_path, array_name)
%PARSE_INT16_C_ARRAY Read a checked-in static const int16_t C array.
%
% This intentionally avoids a single multi-line lazy-regexp capture for the
% initializer body. MATLAB regexp behavior on very large generated headers can
% be brittle across releases; locate the declaration and closing initializer
% explicitly, then parse the comma-separated integer body.

file_path = char(file_path);
array_name = char(array_name);
if isempty(file_path) || isempty(array_name)
    error('hil:profile:CArrayArgument', ...
        'C array file path and array name must be non-empty.');
end

if ~isfile(file_path)
    error('hil:profile:CArrayFileMissing', ...
        'C array source file does not exist: %s.', file_path);
end

text = fileread(file_path);
name = regexptranslate('escape', array_name);
decl_pattern = [ ...
    'static\s+const\s+int16_t\s+' name ...
    '\s*\[\s*\]\s*=\s*\{'];
initializer_start = regexp(text, decl_pattern, 'end', 'once');
if isempty(initializer_start)
    error('hil:profile:CArrayMissing', ...
        'Could not find int16 array "%s" in %s.', array_name, file_path);
end

remainder = text(initializer_start + 1:end);
close_offset = strfind(remainder, '};');
if isempty(close_offset)
    error('hil:profile:CArrayUnterminated', ...
        'Array "%s" in %s has no closing };.', array_name, file_path);
end
body = remainder(1:close_offset(1) - 1);

% Generated profile arrays are numeric initializer lists. Strip comments so a
% future generated/header annotation containing digits cannot be interpreted as
% a current sample.
body = regexprep(body, '/\*[\s\S]*?\*/', ' ');
body = regexprep(body, '//[^\r\n]*', ' ');

% sscanf is deliberately used instead of regexp for the large numeric payload.
% Convert commas to whitespace and require that no non-numeric initializer text
% remains after removing numbers/signs/commas/whitespace.
scan_text = strrep(body, ',', ' ');
values = sscanf(scan_text, '%f');

if isempty(values) || any(~isfinite(values))
    error('hil:profile:CArrayInvalid', ...
        'Array "%s" in %s contains no valid values.', array_name, file_path);
end
if any(values ~= fix(values))
    error('hil:profile:CArrayNonInteger', ...
        'Array "%s" in %s contains a non-integer value.', array_name, file_path);
end
if any(values < double(intmin('int16'))) || any(values > double(intmax('int16')))
    error('hil:profile:CArrayRange', ...
        'Array "%s" in %s contains a value outside int16_t range.', ...
        array_name, file_path);
end

% Detect a partial sscanf parse (for example an unexpected macro/token in the
% initializer) instead of silently returning a truncated drive cycle.
numeric_tokens = regexp(body, '[-+]?\d+', 'match');
if numel(values) ~= numel(numeric_tokens)
    error('hil:profile:CArrayParseMismatch', ...
        ['Array "%s" in %s parsed %d values but contains %d integer tokens; ' ...
         'refusing a partial drive profile.'], ...
        array_name, file_path, numel(values), numel(numeric_tokens));
end

values = double(values(:));
end
