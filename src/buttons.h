#include <stdbool.h>

typedef enum { BUTTONS_TYPE_CW, BUTTONS_TYPE_CCW, BUTTONS_TYPE_STOP, BUTTONS_TYPE_HOME } buttons_type_t;

typedef void (*buttons_handler_t)(buttons_type_t, bool);

void buttons_init(buttons_handler_t);
