#include"AP_ODIDScanner_utils.h"

bool mac_eq(uint8_t a[6], uint8_t b[6]) {
    return a[0] == b[0] &&
           a[1] == b[1] &&
           a[2] == b[2] &&
           a[3] == b[3] &&
           a[4] == b[4] &&
           a[5] == b[5];
}
