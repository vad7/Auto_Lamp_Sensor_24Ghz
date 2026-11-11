# Копирование прошивки после сборки PlatformIO в Build/<окружение>/ (extra_scripts = post:../Scripts/copy_firmware.py).
# RP2040 - firmware.uf2 (копируется на диск RPI-RP2 загрузчика).
import os
import shutil

Import("env")  # noqa: F821 - объект сборки PlatformIO

ROOT = os.path.dirname(os.path.normpath(env.subst("$PROJECT_DIR")))  # noqa: F821 - корень репозитория (над Firmware)
EXTS = (".uf2",)  # форматы прошивки для копирования


def copy_firmware(source, target, env):
    build_dir = env.subst("$BUILD_DIR")
    name = env.subst("$PIOENV")
    out_dir = os.path.join(ROOT, "Build", name)
    os.makedirs(out_dir, exist_ok=True)
    for ext in EXTS:
        src = os.path.join(build_dir, "firmware" + ext)
        if os.path.isfile(src):
            dst = os.path.join(out_dir, "BathAutoLamp_" + name + ext)
            shutil.copyfile(src, dst)
            print("Прошивка скопирована: " + dst)


env.AddPostAction("buildprog", copy_firmware)  # noqa: F821 - после всей сборки (UF2 - отдельный шаг)
