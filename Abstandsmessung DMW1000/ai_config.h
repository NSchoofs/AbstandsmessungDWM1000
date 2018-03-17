#pragma once
#define NR_OF_DRONES 4

//Name der Drohne bei jder Drohne unterschiedlich defined
#define AI_NAME 0 //oder 1;2;3;...
#define AI_ROLE AI_MASTER

#define MAX_HISTORY 0

#define RANGING_FREQUENCY 10 //Frequenz in Hz

//Taskconfig
#define AI_TASK_NAME "ai_task"
#define AI_TASK_STACKSIZE 1024  //kp was genug/zu viel - MIST
#define AI_TASK_PRIO 1			//hier einfaches aber evtl. schlechtes tuning m?glich - Maximum 6, je h?her desto wichtiger (idle hat 0), task der l?uft hat immer die h?chste prio der aktivierbaren tasks

#define PASSIVE_MODE 0			//hier 1 falls drohne nur auf rangen antworten soll
