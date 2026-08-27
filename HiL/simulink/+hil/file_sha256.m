function digest = file_sha256(file_path)
%FILE_SHA256 Compute a SHA-256 digest without loading the whole file.

file_path = char(file_path);
fid = fopen(file_path, 'rb');
if fid < 0
    error('hil:hash:OpenFailed', 'Could not open file "%s".', file_path);
end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>

if ~usejava('jvm')
    error('hil:hash:NoJVM', 'SHA-256 hashing requires the MATLAB JVM.');
end

engine = java.security.MessageDigest.getInstance('SHA-256');
while true
    bytes = fread(fid, 1024 * 1024, '*uint8');
    if isempty(bytes)
        break;
    end
    engine.update(typecast(bytes(:), 'int8'));
end

raw = typecast(int8(engine.digest()), 'uint8');
digest = lower(reshape(dec2hex(raw, 2).', 1, []));
end
