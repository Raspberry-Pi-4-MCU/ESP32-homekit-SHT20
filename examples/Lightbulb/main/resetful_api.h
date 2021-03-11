#ifndef _RESETFUL_API_H_
#define _RESETFUL_API_H_
#include <esp_http_server.h>
#include "App.h"
#include "system_information.h"

httpd_handle_t start_webserver(void);

#endif