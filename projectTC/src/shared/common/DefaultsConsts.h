#ifndef CtrlDefaultConsts_H
#define CtrlDefaultConsts_H

#include <cstdint>

#define SERVER_ROLE                 "AIR"

#define DB_ADDRESS                  "192.168.100.238"
#define DB_PORT                     5432
#define DB_NAME                     "coop"
#define DB_USER                     "dvlp"
#define DB_PASSWORD                 "dvlp"
#define DB_DRIVER                   "QPSQL"

#define SUPPLY_ABONENTS             QStringList({"CP-1.1.1.0"})

constexpr uint16_t MAX_ABONENTS         = 300;

constexpr uint32_t MAX_SRC_TRACKS       = 300;
constexpr uint32_t MAX_BEARINGS         = 50;
constexpr uint32_t MAX_GEN_TRACKS       = 1500;

constexpr uint32_t TASK_PROC_TIMEOUT    = 1000;

constexpr uint16_t MAX_TRACK_ID         = 10000;

#endif
