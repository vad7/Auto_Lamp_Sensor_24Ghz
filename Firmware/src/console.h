// Консоль настройки по USB (монитор порта): чтение и правка калибровки. Readme.md, «Настройка по USB».
#pragma once
#include "board.h"

void calibDefaults(Calib& c);           // настройки по умолчанию (без калибровки) - main.cpp
void calibPlan(Stream& s);              // описание стадий калибровки - main.cpp
bool consoleRun(Stream& s, Calib& c);   // команды до exit / calib или CONSOLE_IDLE_MS без команд; изменения - сразу
                                        // во флеш; true - введена calib: запустить калибровку
