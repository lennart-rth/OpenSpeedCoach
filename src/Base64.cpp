#include "Base64.h"
#include <stdlib.h>
#include <string.h>

static const char b64_table[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

size_t base64_encode_to(const unsigned char* in, size_t len, char* out, size_t outCap) {
    if (!in || !out) return 0;
    size_t required = ((len + 2) / 3) * 4;
    if (outCap <= required) return 0;
    size_t i = 0, o = 0;
    while (i + 2 < len) {
        unsigned a = in[i++];
        unsigned b = in[i++];
        unsigned c = in[i++];
        out[o++] = b64_table[(a >> 2) & 0x3F];
        out[o++] = b64_table[((a & 3) << 4) | ((b >> 4) & 0x0F)];
        out[o++] = b64_table[((b & 15) << 2) | ((c >> 6) & 0x03)];
        out[o++] = b64_table[c & 0x3F];
    }
    if (i < len) {
        size_t rem = len - i;
        if (rem == 1) {
            unsigned a = in[i];
            out[o++] = b64_table[(a >> 2) & 0x3F];
            out[o++] = b64_table[((a & 3) << 4) & 0x3F];
            out[o++] = '=';
            out[o++] = '=';
        } else if (rem == 2) {
            unsigned a = in[i++];
            unsigned b = in[i];
            out[o++] = b64_table[(a >> 2) & 0x3F];
            out[o++] = b64_table[((a & 3) << 4) | ((b >> 4) & 0x0F)];
            out[o++] = b64_table[((b & 15) << 2) & 0x3F];
            out[o++] = '=';
        }
    }
    out[o] = '\0';
    return o;
}

char* base64_encode_alloc(const unsigned char* in, size_t len) {
    size_t required = ((len + 2) / 3) * 4;
    char* out = (char*)malloc(required + 1);
    if (!out) return nullptr;
    base64_encode_to(in, len, out, required + 1);
    return out;
}
