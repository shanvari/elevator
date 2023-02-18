# elevator
microprocessor, CubeIde, C language


در این پروژه قصد داریم سامانهای جهت مدیریت آسانسور طراحی کنیم. این سامانه نیاز دارد تا کارهای اساسی مانند رفتن آسانسور به طبقات
مختلف و توقف در هر طبقه را مدیریت کند. در این سامانه از یک  7-Segmentجهت نمایش طبقه فعلی آسانسور و جهت حرکت آن استفاده
میکنیم. نحوه نمایش طبقات بر روی  7-Segmentبه این شکل است که رقم سمت راست طبقه فعلی را نمایش میدهد و رقم اول از سمت چپ
برای دریافت ورودی استفاده میشود. در اینجا دو رقم وسط لازم نیست و همواره خاموش است.
برای دریافت ورودی از سه دکمه خارجی استفاده میشود. دو دکمه برای کم و زیاد کردن طبقه مورد نظر از صفر تا  9است. کاربر میتواند با
تنظیم طبقه مورد نظر خود بر روی  7-Segmentو فشار دادن دکمه خارجی سوم فرمان لازم برای رفتن به طبقه مورد نظر را صادر کند. فرض کنید
آسانسور در طبقه  ۱است و رقم سمت چپ  7-Segmentدر این لحظه صفر را نمایش میدهد. کاربر میتواند با زیاد کردن این رقم عدد  3را
مشخص کند و با فشار دادن دکمه تایید، فرمان رفتن به طبقه سوم از طبقه اول را صادر نماید. توجه کنید که کمترین رقم ممکن برای رقم سمت
چپ صفر است و بیشترین رقم ممکن آخرین طبقه موجود است. یعنی اگر آسانسور شامل  3طبقه باشد ( 0و  ۱و  ،)2این رقم بین صفر و  2تنظیم
میگردد و حالت چرخشی نیز ندارد. مثلا اگر به صفر برسد دیگر نباید کم شود و دکمه کم کردن رقم بیتاثیر است.
آسانسور باید دارای زنگ خطر نیز باشد. به این منظور نیاز است تا دکمه خارجی برای این کار اضافه شود و بازر به شکلی تنظیم شود تا با
فشردن دکمه، به صورت قطع و وصل شونده به صدا در بیاید. فرکانس قطع و وصل شدن صدای بازر باید  4هرتز باشد. یعنی  2۵0میلیثانیه صدا
تولید میکند و  2۵0میلیثانیه بعدی ساکت است و به همین شکل چرخه خاموش و روشن شدن ادامه پیدا میکند. همچنین در زمان به صدا
در آمدن زنگ باید  LEDهای روی برد را نیز با همین فرکانس خاموش و روشن کنید.
نحوه حرکت آسانسور به این شکل است که طبقات جدید در صفی قرار خواهند گرفت تا به ترتیب ثبت شدن طبقات، به طبقات مورد نظر
برود. در هر طبقه لازم است به اندازه مناسب توقف کند و سپس به حرکت خود ادامه دهد. در اینجا باید به مواردی که ممکن است باعث مشکل
شوند توجه گردد و حالتهای خطرناک به درستی مدیریت شوند. مانند مدیریت مناسب ثبت طبقه تکراری یا ثبت طبقهای که در حال حاضر
آسانسور در آن حضور دارد. در این قسمت سعی شود تا سامانه تاجای امکان مانند آسانسور واقعی عمل کند.
برای آسانسور لازم است حالت ادمینی طراحی شود تا بتوان به کمک دستوراتی از طریق  UARTتنظیمات اولیه را بر روی آسانسور اعمال
کرد. جهت ورود به سامانه ادمین آسانسور، تنها در زمانی که صف طبقات آسانسور خالی است، میتوان از دستور }” “ADMIN#{Passکه
} {Passیک رمز دلخواه حداقل  4کاراکتری است، استفاده کرد. در حالت ادمین میتوان از دستورات زیر جهت تنظیم استفاده نمود:
 :SET MAX LEVEL [N] .۱به کمک این دستور باید بتوان تعداد طبقات آسانسور را تنظیم کرد. برای انجام تنظیم بجای ] [Nشماره
آخرین طبقه قرار میگیرد و مقدار آن بین صفر تا  9است. یعنی اگر این عدد برابر با  3باشد، طبقات آسانسور شامل  2 ،۱ ،0و  3میشود.
توجه کنید که با اعمال این دستور باید طبقه فعلی آسانسور به صفر تغییر پیدا کند.
 :SET LEVEL [N] .2با اجرای این دستور طبقه فعلی آسانسور به مقدار ] [Nتغییر پیدا میکند. عدد دریافتی باید بین صفر و آخرین
طبقه ممکن باشد. توجه کنید که با اجرای این دستور طبقه تنها به شکل منطقی تغییر میکند و اجرای این دستور به معنی حرکت
آسانسور و قرار گیری آن طبقه در صف نیست.
 :SET WAIT [N] .3با این دستور زمان توقف آسانسور در هر طبقه به میلیثانیه تنظیم میگردد. در اینجا ] [Nباید ضریب  ۱00و بین
 ۵00تا  ۵000تنظیم گردد.
 :SET LED {ON/OFF} .4با اجرای این دستور میتوان  LEDهای روی برد را در هنگام زنگ خطر قطع یا وصل کرد. در صورت قطع
بودن دیگر با فشار دادن دکمه خطر،  LEDهای روی برد روشن نمیشوند.
توجه کنید که اگر ورودی صحیحی وارد نشود، دستور نباید عمل کند. مثلا اگر از دستور  2جهت تنظیم طبقه آسانسور به  8استفاده شود در
حالی که بیشترین طبقه ممکن  7است این دستور باید خطا بدهد. همچنین لازم است برای اجرای هر دستور پیغامی مناسب بر روی UART
نشان داده شود.
دستور مهم دیگری که در حالت ادمین وجود دارد، تست آسانسور است. کاربر باید بتواند با دستور }” “TEST#{sampleکه }{sample
شامل رشتهای از ارقام مجاز برای طبقه آسانسور (حداقل صفر و حداکثر آخرین طبقه) صف آسانسور را پر نماید. با اجرای این دستور باید وضعیت
صف آسانسور در ترمینال نمایش داده شود تا از وضعیت صف قبل از اجرای آسانسور مطمئن شویم. در اینجا } {sampleحداقل یک رقم است و
حداکثر طول آن  ۵رقم است و ترتیب طبقات نیز از چپ به راست است. یعنی در دستور  TEST#102ابتدا  ۱در صف قرار میگیرد، سپس صفر
و در نهایت طبقه  .2با اجرای دستور ” “STARTآسانسور از حالت ادمین خارج میگردد و در صورتی که صف آن به کمک دستور تست پر شده
بود، حرکت لازم را انجام میدهد. (اگر پس از پر کردن صف به کمک دستور تست، مجددا از دستورات  ۱یا  2در بالا استفاده شود، صف خالی
میشود)
نمر
