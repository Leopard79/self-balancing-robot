function readAndPlot(src, h)
    % persistent لتذكر وقت البداية بين الاستدعاءات المتتالية
    persistent t_start;
    if isempty(t_start)
        t_start = tic; 
    end
    
    try
        % قراءة السطر القادم من الآردوينو
        raw = readline(src);
        val = str2double(raw);
        
        if ~isnan(val)
            elapsed = toc(t_start);
            % إضافة النقطة الجديدة للرسم
            addpoints(h, elapsed, val);
            title(sprintf('الزاوية الحالية = %.1f°', val));
            drawnow limitrate;
        end
    catch
        % لتجنب توقف البرنامج في حال حدوث خطأ عابر في السيريال
    end
end