function [] = export_fun(fun, src_path, matlab_convert_path)
% This function MUST be called from the root directory of the project!!

fun.export(src_path, ...
    'TemplateHeader', [pwd, '/MATLAB/symbolic/Template/template_export.hh'], ...
    'TemplateFile',   [pwd, '/MATLAB/symbolic/Template/template_export.cc']);
convertFile(fun.Name, src_path, matlab_convert_path)

end

