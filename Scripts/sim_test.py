"""Симулятор прошивки на ПК: сборка исходников Firmware/src с заглушкой платы (Tests/sim) обычным g++ и прогон
сценариев (Tests/sim/scenarios.cpp) в виртуальном времени.

Запуск из корня проекта:
    python Scripts/sim_test.py              - все варианты и сценарии
    python Scripts/sim_test.py stuck_jar    - сценарии, в имени которых есть строка
    python Scripts/sim_test.py -v ...       - полный журнал каждого сценария в консоль
    python Scripts/sim_test.py --src=DIR    - исходники прошивки из DIR (проверить правку на копии)

Журналы - work/sim/<вариант>_<сценарий>.log. Нужен g++ (MinGW) в PATH; санитайзеров в MinGW нет.
Что симулятор НЕ проверяет: тайминги UART, сон с остановкой тактов, флеш, USB, АЦП и шум фоторезистора;
модель датчика - по перехвату (work/ld2410s_sniff_notes.md), не по железу.
"""
import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "work", "sim")
FW_SOURCES = ["main.cpp", "console.cpp", "ld2410s.cpp"]   # из Firmware/src (board_rp2040.cpp - заменяет sim_core)
SIM_SOURCES = ["Tests/sim/sim_core.cpp", "Tests/sim/scenarios.cpp"]
FLAGS = ["-std=gnu++11",   # не c++11: в MinGW 4.8 строгий режим ломает math.h (_hypot)
         "-O1", "-g", "-Wall", "-Wextra", "-D__USE_MINGW_ANSI_STDIO=1", "-DARDUINO_ARCH_RP2040", "-ITests/sim"]
VARIANTS = [("lamp", []), ("night", ["-DNIGHT_LIGHT"]), ("debug", ["-DDEBUG_LOG"])]   # как окружения platformio.ini
RUN_TIMEOUT_S = 300   # один сценарий, с реального времени


def main():
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    args = sys.argv[1:]
    verbose = "-v" in args
    src = next((a[len("--src="):] for a in args if a.startswith("--src=")), "Firmware/src")
    filters = [a for a in args if a != "-v" and not a.startswith("--src=")]
    sources = [os.path.join(src, f) for f in FW_SOURCES] + SIM_SOURCES
    os.makedirs(OUT, exist_ok=True)
    total = failed = 0
    for name, defs in VARIANTS:
        exe = os.path.join(OUT, "sim_%s.exe" % name)
        build = subprocess.run(["g++"] + FLAGS + ["-I" + src] + defs + sources + ["-o", exe], cwd=ROOT,
                               capture_output=True)
        text = build.stderr.decode("utf-8", "replace").strip()
        if build.returncode != 0:
            print("[%s] СБОРКА НЕ УДАЛАСЬ:\n%s" % (name, text))
            failed += 1
            continue
        print("[%s] собрано%s" % (name, (", предупреждения:\n" + text) if text else " без предупреждений"))
        names = subprocess.run([exe, "list"], capture_output=True).stdout.decode().split()
        for sc in names:
            if filters and not any(f in sc for f in filters):
                continue
            total += 1
            try:
                run = subprocess.run([exe, sc], capture_output=True, timeout=RUN_TIMEOUT_S)
                log = run.stdout.decode("utf-8", "replace")
                code = run.returncode
            except subprocess.TimeoutExpired as e:
                log = (e.stdout or b"").decode("utf-8", "replace") + "\nЗАВИСАНИЕ: дольше %d с\n" % RUN_TIMEOUT_S
                code = -1
            path = os.path.join(OUT, "%s_%s.log" % (name, sc))
            with open(path, "w", encoding="utf-8") as f:
                f.write(log)
            ok = code == 0
            failed += 0 if ok else 1
            summary = [l for l in log.splitlines() if l.startswith("ИТОГ")]
            print("  %-20s %s  %s" % (sc, "ок    " if ok else "ПРОВАЛ", summary[0] if summary else "код %d" % code))
            if verbose:
                print(log)
            elif not ok:
                for line in log.splitlines():
                    if "ПРОВАЛ" in line and not line.startswith("ИТОГ") or "ЗАВИСАНИЕ" in line or "НАРУШЕНИЕ" in line:
                        print("      " + line)
                print("      журнал: %s" % os.path.relpath(path, ROOT))
    print("Сценариев %d, провалено %d" % (total, failed))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
