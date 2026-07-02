function export_figure(name_base, opts)
% export_figure  Write PNG + EPS sidecars with the project's standard options.
%   export_figure("figs/TO_01_kinetic_energy")
%   export_figure("figs/TO_07_heatmap", EPSContentType="image")  % surf-heavy figs
%   export_figure("figs/foo", Width=2400, Height=1400)
%   export_figure("figs/foo", Fig=my_fig)                        % explicit target
arguments
    name_base string
    opts.Fig (1,1) matlab.ui.Figure = gcf
    opts.Width  double = 2000
    opts.Height double = 1400
    opts.EPSContentType (1,1) string = "vector"
end
exportgraphics(opts.Fig, name_base + ".png", ContentType="image", ...
    Width=opts.Width, Height=opts.Height, Resolution=300, Padding=40);
exportgraphics(opts.Fig, name_base + ".eps", ContentType=opts.EPSContentType, ...
    Width=opts.Width, Height=opts.Height, Resolution=300, Padding=40);
end
