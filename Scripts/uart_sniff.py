# Перехват обмена программы HLK-LD2410S_TOOL с датчиком: мост между виртуальным портом (пара com0com) и реальным
# адаптером USB-UART, весь трафик в обе стороны - в журнал work/uart_sniff_<время>.log (HEX, по кадрам).
#   python Scripts/uart_sniff.py <реальный порт> <виртуальный порт> [скорость]
#   пример: python Scripts/uart_sniff.py COM7 COM21     (программа HLK подключается ко второму порту пары, COM20)
# Кадры (протокол LD2410S, табл. 2-1, п. 2.2): команда / ACK FD FC FB FA ... 04 03 02 01, стандартный отчёт
# F4 F3 F2 F1 ... F8 F7 F6 F5, минимальный отчёт 6E ... 62 (5 байт). Требуется: pyserial.
import datetime
import os
import sys
import threading
import time

import serial

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CMD_TAIL = bytes([0x04, 0x03, 0x02, 0x01])   # конец команды и ACK
REP_TAIL = bytes([0xF8, 0xF7, 0xF6, 0xF5])   # конец стандартного отчёта
MIN_HEAD, MIN_LEN = 0x6E, 5                  # минимальный отчёт
IDLE_FLUSH_S = 0.05                          # пауза в потоке - вывести накопленное, даже без конца кадра

lock = threading.Lock()


def log(out, who, data):
    line = "%s %s %s" % (datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3], who, data.hex(" ").upper())
    with lock:
        print(line)
        out.write(line + "\n")
        out.flush()


def pump(src, dst, who, out, skip_min):
    """Пересылает байты src -> dst, режет на кадры по концам кадров и пишет в журнал."""
    buf = bytearray()
    last = time.time()
    while True:
        data = src.read(src.in_waiting or 1)
        if data:
            dst.write(data)
            buf += data
            last = time.time()
            while True:
                if buf and buf[0] == MIN_HEAD and len(buf) >= MIN_LEN:
                    if not skip_min:
                        log(out, who, bytes(buf[:MIN_LEN]))
                    del buf[:MIN_LEN]
                    continue
                ends = [buf.find(t) for t in (CMD_TAIL, REP_TAIL)]
                ends = [e for e in ends if e >= 0]
                if not ends:
                    break
                e = min(ends) + 4
                log(out, who, bytes(buf[:e]))
                del buf[:e]
        elif buf and time.time() - last > IDLE_FLUSH_S:
            log(out, who + "?", bytes(buf))
            buf.clear()


def main():
    if len(sys.argv) < 3:
        print(__doc__ or "python Scripts/uart_sniff.py <реальный порт> <виртуальный порт> [скорость]")
        sys.exit(1)
    args = [a for a in sys.argv[1:] if not a.startswith("--")]   # позиционные аргументы без ключей
    real, virt = args[0], args[1]
    baud = int(args[2]) if len(args) > 2 else 115200
    skip_min = "--all" not in sys.argv   # минимальные отчёты (4 в секунду) не писать, кроме --all
    os.makedirs(os.path.join(ROOT, "work"), exist_ok=True)
    path = os.path.join(ROOT, "work", "uart_sniff_%s.log" % datetime.datetime.now().strftime("%Y%m%d_%H%M%S"))
    a = serial.Serial(real, baud, timeout=0.01)
    b = serial.Serial(virt, baud, timeout=0.01)
    with open(path, "w", encoding="utf-8") as out:
        print("Журнал:", path, "| PC> - программа -> датчик, LD> - датчик -> программа; Ctrl+C - стоп")
        threading.Thread(target=pump, args=(b, a, "PC>", out, skip_min), daemon=True).start()
        threading.Thread(target=pump, args=(a, b, "LD>", out, skip_min), daemon=True).start()
        try:
            while True:
                time.sleep(0.5)
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
