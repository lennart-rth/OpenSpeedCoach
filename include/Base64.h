#ifndef BASE64_H
#define BASE64_H

#include <cstddef>

// Encode `len` bytes from `in` into a newly allocated null-terminated
// C string. Caller must free() the returned pointer. Returns nullptr on
// allocation failure.
char* base64_encode_alloc(const unsigned char* in, size_t len);

// Encode a slice to the provided output buffer. `out` must have space
// for at least ((len+2)/3*4)+1 bytes. Returns number of bytes written
// (excluding NUL).
size_t base64_encode_to(const unsigned char* in, size_t len, char* out, size_t outCap);

#endif
