#ifndef HOST_STUB_ESP_LOG_H_
#define HOST_STUB_ESP_LOG_H_

#include "esp_err.h"

#define ESP_LOGI(tag, format, ...) ((void)(tag))
#define ESP_LOGW(tag, format, ...) ((void)(tag))
#define ESP_LOGE(tag, format, ...) ((void)(tag))

#endif /* HOST_STUB_ESP_LOG_H_ */
