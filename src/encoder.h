#include <stdint.h>

typedef void (*encoder_handler_t)(double);

void encoder_init(encoder_handler_t);
