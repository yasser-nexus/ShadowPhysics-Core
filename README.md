# FH4 Realistic Physics Overhaul (FH4RPO)

مود C++ كامل (ASI / DLL) لـ **Forza Horizon 4** يعيد بناء طبقة الفيزياء: مصفوفة احتكاك ديناميكية حسب
زاوية الانزلاق، نقل وزن ديناميكي، قوة ضغط هوائي محسوبة من مربع السرعة، ونموذج إطارات Pacejka،
كل ذلك فوق **محرك Hooking آمن ذاتي التنظيف (MinHook)** وطبقة ذاكرة محمية بـ SEH.

المشروع **قابل للبناء والتجميع فعلياً** — تم التحقق منه في هذه البيئة:
* بناء كامل لوحدة Windows (`.asi` + `.dll`) عبر MinGW-w64 x64 ✔
* فحص ترجمة لكل الملفات على مسار `clang-cl` (SEH حقيقي) ✔
* 38 مجموعة اختبار للفيزياء تعمل وتنجح: `ALL TESTS PASSED` ✔

---

## 1. الملفات

```
FH4RPO/
├── CMakeLists.txt                  ← ملف البناء (يولّد FH4RPO.asi + FH4RPO.dll)
├── README.md                        ← هذا الملف
├── assets/
│   └── FH4RPO.ini                   ← الإعدادات الكاملة (تُولَّد من كود الافتراضيات)
├── include/fh4rpo/
│   ├── PhysicsEngine.hpp            ← واجهة محرك الفيزياء (المطلوب #1)
│   ├── Core.hpp                     ← رياضيات، جداول Bilinear، طابور SPSC، Clamp/NaN
│   ├── Config.hpp                   ← كل قابل للتعديل + إعادة تحميل حيّة
│   ├── VehicleState.hpp             ← حالة العجلات/الهيكل/النتائج
│   ├── Telemetry.hpp                ← هيكل حزمة Data Out (324 بايت) + مستقبل UDP
│   ├── MemoryGuard.hpp              ← قراءة/كتابة آمنة + SEH + Watchdog
│   ├── HookEngine.hpp               ← غلاف MinHook آمن + Auto-Unhook
│   ├── PatternScanner.hpp           ← مسح التواقيع (AOB) محمي
│   ├── GameBinds.hpp                ← طبقة العناوين (توقيع / RVA / سلسلة مؤشرات / تثبيت يدوي)
│   ├── Module.hpp                   ← دورة حياة الوحدة + Hotkeys
│   └── Log.hpp                      ← لوغ آمن للخيوط
├── src/
│   ├── PhysicsEngine.cpp            ← منطق المعادلات الكامل (المطلوب #2)
│   ├── Hooks.cpp                    ← الـ Detours الخمسة (جزء من #2، فُصل للوضوح)
│   ├── dllmain.cpp                  ← نقطة الدخول الآمنة (المطلوب #3)
│   ├── Module.cpp                   ← التشغيل/الإيقاف/المفاتيح/Watchdog
│   ├── Config.cpp                   ← قارئ/كاتب INI بدون أي مكتبة خارجية
│   ├── MemoryGuard.cpp              ← SEH + VirtualQuery + Panic filter
│   ├── HookEngine.cpp               ← MinHook wrapper + تنظيف عكسي
│   ├── PatternScanner.cpp           ← ماسح AOB سريع (memcmp + wildcard)
│   ├── GameBinds.cpp                ← حل وتحقّق العناوين
│   ├── Telemetry.cpp                ← مستقبل UDP + فك ترميز محصّن
│   └── Log.cpp
├── third_party/minhook/             ← MinHook (BSD-2) مضمَّن في الشجرة
├── tests/PhysicsTests.cpp           ← اختبارات الفيزياء (تعمل بدون اللعبة)
├── tools/GenDefaultIni.cpp          ← توليد assets/FH4RPO.ini من الكود
└── cmake/toolchain-mingw-w64.cmake  ← بناء عابر من Linux
```

> ملاحظة على تسمية الملفات: طلبتَ 4 ملفات؛ الهيكل هنا نفس المحتوى لكن مقسّماً بشكل إنتاجي
> (`PhysicsEngine.cpp` + `Hooks.cpp` + `Module.cpp`) لأن وضع 2500 سطر في ملف واحد يجعل
> دورة تصحيح أخطاء الـ Hook بطيئة جداً. كل ما طلبته موجود، ولم يُحذف أي شيء.

---

## 2. البناء

### Windows — Visual Studio 2019/2022 (المسار المفضّل، SEH حقيقي)
```bat
cmake -S . -B build -G "Visual Studio 17 2022" -A x64
cmake --build build --config Release
:: الناتج: build\dist\FH4RPO.asi + FH4RPO.dll + FH4RPO.ini
```

### Windows — clang-cl (SEH حقيقي أيضاً)
```bat
set CC=clang-cl
set CXX=clang-cl
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

### Linux — بناء عابر للتحقق (MinGW-w64)
```bash
cmake -S . -B build-win -G Ninja \
      -DCMAKE_TOOLCHAIN_FILE=cmake/toolchain-mingw-w64.cmake \
      -DCMAKE_BUILD_TYPE=Release
cmake --build build-win          # ينتج build-win/dist/FH4RPO.asi
```
> MinGW/GCC لا يملك SEH بأسلوب MSVC؛ في هذا المسار تتحول الحُرّاس تلقائياً إلى `try/catch`
> (`FH4RPO_SEH == 0`). للنسخة النهائية استخدم MSVC أو clang-cl.

### اختبارات الفيزياء (أي نظام)
```bash
cmake -S . -B build && cmake --build build
ctest --test-dir build --output-on-failure     # أو: ./build/bin/fh4rpo_tests
```

---

## 3. التركيب

1. ثبّت **ASI Loader** (أو استخدم `dinput8.dll` من Ultimate ASI Loader).
2. انسخ `FH4RPO.asi` + `FH4RPO.ini` إلى مجلد اللعبة (بجانب `ForzaHorizon4.exe`).
3. في اللعبة: **Settings → HUD and Gameplay → Data Out**:
   * `Data Out` = **ON**
   * `Data Out IP` = **127.0.0.1**
   * `Data Out Port` = **5300** (أو غيّرها في `FH4RPO.ini`)
4. شغّل اللعبة. تحقق من `FH4RPO.log` بجانب المود.

**المفاتيح:** `F2` تشغيل/إيقاف · `F3` الوضع السلبي · `F4` إيقاف طارئ لكل الـ Hooks ·
`F5` إعادة تحميل الإعدادات · `F8` كتابة تفريغ حالة (`FH4RPO_state.txt`).

---

## 4. كيف يعمل — خط الأنابيب

```
Data Out (UDP 5300)
        │  Telemetry.cpp: فك ترميز محصّن (كل قيمة تُمرَّر على ClampFinite)
        ▼
  TelemetrySample ──(SPSC ring، بدون أي lock في مسار اللعبة)──► PhysicsEngine::OnFrame(dt)
        │
        ├─ 1. Weight Transfer      dFz = m·a·h / L  ,  split أمامي/خلفي حسب صلابة اللف
        ├─ 2. Tyre model           Magic Formula + Friction Ellipse + load sensitivity
        ├─ 3. Friction Matrix      جدول Bilinear (زاوية انزلاق × حمل عمودي)
        ├─ 4. Aero                 F = ½ρv²·C·A  (+ ground effect، + stall، + drag)
        ├─ 5. Drivetrain/Brakes    منحنى عزم، TCS، ABS، fade
        ├─ 6. Suspension           k·x + c·v  لكل زاوية
        └─ 7. Integrate + Attitude (pitch/roll/yaw targets)
        │
        ▼
  Write-back آمن (MemoryGuard::WriteFloatAtomic) → downforce / grip scale / brake bias /
  steer angle / مصفوفة الاحتكاك داخل حالة السيارة، + تفريغ تشخيصي.
```

### الـ Hooks (5)

| Hook | الشكل | الوظيفة |
|---|---|---|
| `hook_friction_matrix` | `float __fastcall(void*, void*, int, float, float)` | يضرب ناتج اللعبة في مُضاعِف الجدول حسب زاوية الانزلاق والحمل |
| `hook_downforce` | `float __fastcall(void*, void*, float, float)` | يستبدل قيمة الضغط الهوائي بحساب ½ρv²CA |
| `hook_suspension_solve` | `void __fastcall(void*, void*, float)` | يحدّث جدول عناوين الكتابة عبر المُحلّل |
| `hook_tire_force` | `void __fastcall(void*, void*, float)` | يعيد حساب مُضاعِف الاحتكاك داخل مسار القوة |
| `hook_present` | `HRESULT __stdcall(IDXGISwapChain*, UINT, UINT)` | ساعة الإطار الدقيقة (QPC) لخطوة الفيزياء |

`hook_present` يُبنى من جدول دوال `IDXGISwapChain` (D3D11) فيعمل على أي إصدار من اللعبة،
بينما الأربعة الأخرى تُحلّ بالتواقيع من داخل `ForzaHorizon4.exe`.

---

## 5. هندسة "Zero-Crash" — أربع طبقات

1. **SEH على كل وصول للذاكرة**: `MemoryGuard::Read/Write/ReadBlock/WriteBlock/SafeCall*`
   كلها `__try/__except`، وأجسام الـ Hooks ومراحل الحل كلها مُغلّفة كذلك
   (خطأ في إطار واحد ⇒ إطار واحد يُتجاهل، لا انهيار).
2. **تحقق مسبق من الصلاحيات**: `VirtualQuery` + أعلام الحماية قبل كل كتابة، فالصفحات
   المحمية تُرفض دون إثارة استثناء أصلاً.
3. **Watchdog عملية كاملة**: `SetUnhandledExceptionFilter` يعترض أي استثناء هارب،
   يسجّله ويُقفل الوحدة على وضع **Passthrough** بدل قتل اللعبة.
4. **Auto-Unhook حقيقي**: `HookEngine::Shutdown()` يعيد المرور على كل الـ Hooks بترتيب
   عكسي → `MH_DisableHook` → `MH_RemoveHook` → `MH_Uninitialize`، ويُصفّر كل مؤشر
   Trampoline. يُستدعى من مسار الإنهاء الطبيعي **ومن** `DllMain(DLL_PROCESS_DETACH)`،
   فلا يبقى أي pointer عالق في ذاكرة اللعبة.

إضافات:
* **Single instance** عبر Mutex مُسمّى (لا تحميل مزدوج).
* **`DllMain` نظيف**: لا I/O ولا انتظار تحت قفل المُحمّل؛ خيط عامل هو من يبني كل شيء.
* **عند إنهاء العملية** (`reserved != nullptr`) لا ينتظر المود — يفك الـ Hooks فقط (هذا
  ما يمنع الكراش الكلاسيكي عند الخروج).
* **حدود أمان**: `ClampFinite` على كل قيمة تدخل الحل، `SafeDiv` بدل القسمة على صفر،
  مُدمج Fixed-Step بسقف substeps (لا "دوامة موت")، ومُحدِّد معدل على كل كميات المُشغِّل.

---

## 6. المعادلات الأساسية

| الكمية | المعادلة | الملف |
|---|---|---|
| نقل الوزن الطولي | `ΔFz = m·ax·h / L` | `SolveWeightTransfer` |
| نقل الوزن العرضي | `ΔFz = m·ay·h / T`، موزّع بنسبة صلابة اللف والـ ARB | `SolveWeightTransfer` |
| الحمل العمودي للزاوية | `Fz_static ± ΔFz_long ± ΔFz_lat`، مع حد أدنى وحماية من الحمل السالب | `SolveWeightTransfer` |
| قوة الإطار الطولية/العرضية | `F = D·sin(C·atan(B·s − E·(B·s − atan(B·s))))` | `TireMagicFormula` |
| حساسية الحمل | `μ = μ0·(1 − k·(Fz/Fz0 − 1))` | `EvaluateTires` |
| القطع الناقص للاحتكاك | `(Fx/Fx_max)² + (Fy/Fy_max)² ≤ 1` | `EvaluateTires` |
| الضغط الهوائي | `F = ½·ρ·v²·C_L·A` × ground-effect × stall | `SolveAero` / `EvaluateDownforce` |
| المقاومة الهوائية | `F_d = ½·ρ·v²·C_d·A` | `SolveAero` |
| القوة الجانبية من الانزلاق | `F_y = k_yaw·½ρv²A·sin(β)` | `SolveAero` |
| عزم اللف/الانحناء | `M = Fz·h` ⇒ هدف زاوية الهيكل عبر الصلابة | `SolveWeightTransfer` |
| مصفوفة الاحتكاك | `μ_mult = Table(|β| , Fz)` ثم تنعيم أسّي بمعدل `fade_rate` | `ApplyFrictionMatrix` |

---

## 7. الصدق التقني (مهم)

* **العناوين**: لا يوجد مود حقيقي يعرف عناوين ثابتة داخل `ForzaHorizon4.exe` لكل إصدار.
  لهذا المشروع **لا يعتمد على أرقام سحرية**: كل عنوان يُحلّ بمسح توقيع (AOB) أو سلسلة
  مؤشرات أو RVA، وكل عقد (`Bind`) يُتحقّق منه قبل الاستخدام. إن فشل توقيع بعد تحديث
  اللعبة، تتعطّل **ميزة واحدة** فقط ويُخبرك اللوج بالضبط أي واحدة، ويبقى المود يعمل.
* **مخرج الطوارئ**: قسم `[Addresses]` في `FH4RPO.ini` يتيح تثبيت العناوين يدوياً (hex)
  دون إعادة بناء — إن حدّثت اللعبة وتغيّرت التواقيع.
* **المسار المضمون دائماً**: حتى بدون أي Hook (لا توقيع مطابق)، يعمل محرك الفيزياء
  كاملاً من بث `Data Out` ويضخ نتائجه إلى أي عنوان متاح — ويبقى اللوج يقول `telemetry`.
* **الاستخدام**: للعب الفردي/الأوفلاين. لا تستخدمه في أي جلسة أونلاين أو إعلانات/فعاليات
  تنافسية. لا يوجد هنا تجاوز لحماية أو DRM؛ كل ما يفعله تغيير أرقام فيزياء في الذاكرة.
* **التوافق**: بعض تحسينات المود تفترض أن اللعبة تُطبّق ضغطاً هوائياً بمقدار معيّن؛ إن
  كانت سيارتك شرعية تماماً (بدون أجنحة) ستلاحظ فرقاً أقل. عدّل `[Aero]` لتضبطه.

---

## 8. استكشاف الأخطاء

| العرض | السبب المحتمل | الحل |
|---|---|---|
| لا يوجد ملف log | اللودر لا يحمّل الـ ASI | ثبّت Ultimate ASI Loader، وتأكد من معمارية x64 |
| `Scanner: ... not found` | تحديث لعبة / توقيع مختلف | استخدم قسم `[Addresses]` لتثبيت العنوان |
| `telemetry_live=0` في التفريغ | Data Out مغلق أو بورت مختلف | فعّل Data Out واضبط 127.0.0.1:5300 |
| `guard tripped` متكرر | عنوان قديم/محمي | يُطفأ تلقائياً بعد `max_guard_trips`، راجع اللوج |
| شعور غير طبيعي | جداول غير مضبوطة لسيارتك | `[Vehicle]` الوزن و`[FrictionMatrix]` والـ `global_grip_scale` |

للضبط: شغّل اللعبة، اضغط `F8`، واقرأ `FH4RPO_state.txt` (حمل كل زاوية، μ، المُضاعِف،
الـ downforce، cost بالميكروثانية لكل إطار).

---

## 9. الترخيص

كود المشروع: MIT. **MinHook**: BSD-2-Clause (انظر `third_party/minhook/LICENSE.txt`).
Forza Horizon 4 علامة تجارية لـ Microsoft / Playground Games؛ هذا المشروع غير مرتبط بهم.
