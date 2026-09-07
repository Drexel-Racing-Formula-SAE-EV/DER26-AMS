function test_embedded_profile_parser()
%TEST_EMBEDDED_PROFILE_PARSER Verify all checked-in drive-cycle C arrays.
paths = hil.project_paths();
header = fullfile(paths.esp32_plant,'main','drive_profiles.h');
cases = { ...
    'udds25_i_10ma', 14001, 2, 109; ...
    'us06_25_i_10ma', 6001, 2, 15; ...
    'la92_25_i_10ma', 14351, 2, 14};
for k = 1:size(cases,1)
    values = hil.parse_int16_c_array(header,cases{k,1});
    assert(numel(values) == cases{k,2});
    assert(values(1) == cases{k,3});
    assert(values(end) == cases{k,4});
    assert(all(isfinite(values)));
    assert(all(values >= -32768 & values <= 32767));
end
fprintf('PASS: embedded drive profile parser (%d arrays)\n',size(cases,1));
end
