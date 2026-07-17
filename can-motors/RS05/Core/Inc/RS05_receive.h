#ifndef RS05_RECEIVE_H
#define RS05_RECEIVE_H

#include "RS05_app.h"

/* 第一层按通信类型分发，第二层按参数 index 分发。 */
void RS05_ParseFeedback(RS05_MotorTypedef *motor,
                        uint32_t ext_id,
                        const uint8_t data[8]);
void RS05_ParseParameterReply(RS05_MotorTypedef *motor,
                              uint32_t exd_id,
                              const uint8_t data[8]);
void RS05_ProcessFrame(RS05_ManagerTypedef *manager,
                       uint32_t ext_id,
                       const uint8_t data[8]);

#endif /* RS05_RECEIVE_H */
