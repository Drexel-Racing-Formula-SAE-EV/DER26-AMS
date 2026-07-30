function cfg = dataset(name)
%DATASET Load a named dataset configuration.
cfg = hil.load_configuration('dataset', name);
end
