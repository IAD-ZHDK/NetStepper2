#include <stdint.h>

typedef void (*enc_handler_t)(double);

void enc_init(enc_handler_t);
