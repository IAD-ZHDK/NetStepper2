#include <stdbool.h>

typedef enum { END_STOP_PIN_1, END_STOP_PIN_2 } end_stop_pin_t;

typedef void (*end_stop_handler_t)(end_stop_pin_t, bool);

void end_stop_init(end_stop_handler_t);
