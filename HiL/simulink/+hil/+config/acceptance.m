function cfg = acceptance(name)
%ACCEPTANCE Load named temperature-aware validation acceptance limits.
cfg = hil.load_configuration('acceptance', name);
end
