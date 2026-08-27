function cfg = simulation(name)
%SIMULATION Load a named simulation configuration.
cfg = hil.load_configuration('simulation', name);
end
