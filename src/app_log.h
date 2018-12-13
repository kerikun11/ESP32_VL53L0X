#pragma once

#include <iomanip>
#include <iostream>

#if 1
#define logd                                                                   \
  (std::cout << "[D] " << std::setw(8) << std::setfill(' ')                    \
             << xTaskGetTickCount() * portTICK_PERIOD_MS << " [" << __FILE__   \
             << ":" << __LINE__ << "] [" << __func__ << "()] ")
#else
#define logd
#endif

#if 1
#define logi                                                                   \
  (std::cout << "[I] " << std::setw(8) << std::setfill(' ')                    \
             << xTaskGetTickCount() * portTICK_PERIOD_MS << " [" << __FILE__   \
             << ":" << __LINE__ << "] [" << __func__ << "()] ")
#else
#define logi
#endif

#if 1
#define logw                                                                   \
  (std::cout << "[W] " << std::setw(8) << std::setfill(' ')                    \
             << xTaskGetTickCount() * portTICK_PERIOD_MS << " [" << __FILE__   \
             << ":" << __LINE__ << "] [" << __func__ << "()] ")
#else
#define logw
#endif

#if 1
#define loge                                                                   \
  (std::cout << "[E] " << std::setw(8) << std::setfill(' ')                    \
             << xTaskGetTickCount() * portTICK_PERIOD_MS << " [" << __FILE__   \
             << ":" << __LINE__ << "] [" << __func__ << "()] ")
#else
#define loge
#endif
