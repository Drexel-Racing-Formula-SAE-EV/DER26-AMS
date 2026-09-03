function report = fuse(replay,acceptance)
%FUSE Enforce conservative production/reference fuse-model relationships.
state_nonconservative=max(0,replay.reference_utilization-replay.production_utilization);
cap_nonconservative=max(0,replay.production_cap_A-replay.reference_cap_A);
report=struct();
report.max_state_nonconservative=max(state_nonconservative,[],'all');
report.max_cap_nonconservative_A=max(cap_nonconservative,[],'all');
report.production_underestimate_fraction=mean( ...
    replay.production_utilization>=replay.reference_utilization- ...
    acceptance.state_nonconservative_tolerance);
report.strict_replay_pass=logical(replay.strict_pass);
report.pass=report.strict_replay_pass && ...
    report.max_state_nonconservative<=acceptance.state_nonconservative_tolerance && ...
    report.max_cap_nonconservative_A<=acceptance.cap_nonconservative_tolerance_A;
report.note='Model-consistency evidence only until installed EAC14-80 path is characterized.';
end
