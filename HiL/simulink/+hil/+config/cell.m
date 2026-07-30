function cfg = cell(name)
%CELL Load a named cell configuration.
cfg = hil.load_configuration('cell', name);
end
