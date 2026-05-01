#ifndef APP_H
#define APP_H

void App_Init(void);

/* Registers periodic tasks then returns; call before main loop. */
void App_StartScheduling(void);

#endif
