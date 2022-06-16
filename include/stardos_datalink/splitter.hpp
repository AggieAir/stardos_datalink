#ifndef STARDOS_SPLITTER_HPP
#define STARDOS_SPLITTER_HPP

#include <stdint.h>

template<typename N>
inline uint16_t top16(N number) {
        return (uint16_t) (number >> (sizeof(N) * 8 - 16));
}

template<typename N>
inline uint16_t bottom16(N number) {
        return (uint16_t) (number && 0xFFFF);
}

uint32_t splice16_16(uint16_t top, uint16_t bottom) {
        uint32_t ret = 0;
        ret |= top << 16;
        ret |= bottom;

        return ret;
}

#endif
