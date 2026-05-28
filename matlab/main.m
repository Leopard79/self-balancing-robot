%% 1. فتح الاتصال بالروبوت
myport = serialport('COM4', 115200);
configureTerminator(myport, "LF");
pause(2);
flush(myport);

%% 2. إعداد نافذة الرسم البياني
figure('Name','استجابة روبوت التوازن');
h = animatedline('Color','b','LineWidth',1.5);
yline(177.4,'r--','Setpoint','LineWidth',1.5);
grid on;
xlabel('الزمن (ثانية)');
ylabel('الزاوية (درجة)');

%% 3. تفعيل القراءة والرسم في الخلفية (Callback)
% هنا نخبر ماتلاب: فور وصول سطر جديد من الآردوينو، نفذ دالة الرسم فوراً
configureCallback(myport, "terminator", @(src, event) readAndPlot(src, h));

disp('✓ الروبوت متصل والنافذة جاهزة.. يمكنك التعديل الآن من الـ Command Window!');