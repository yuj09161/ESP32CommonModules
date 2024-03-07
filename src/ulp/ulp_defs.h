#ifndef __ULP_DEF_H_INCLUDED__
#define __ULP_DEF_H_INCLUDED__

typedef enum {
    ULP_SUCCESS,
    ULP_ERR_TIMEOUT,
} ErrCodes;

typedef enum {
    ULP_STATUS_DHT11_REQUESTED = 0x01,
    ULP_STATUS_DHT11_COMPLETED = 0x02
} ULPStatus;

#endif