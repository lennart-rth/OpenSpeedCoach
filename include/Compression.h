#ifndef COMPRESSION_H
#define COMPRESSION_H

#include <cstddef>
#include <zlib.h>

class StreamingCompressor {
public:
	StreamingCompressor();
	~StreamingCompressor();

	bool begin(int level = Z_BEST_COMPRESSION, int windowBits = MAX_WBITS);
	int feed(const unsigned char* input,
			 size_t inputLen,
			 unsigned char* output,
			 size_t outputCap,
			 size_t* consumed,
			 size_t* produced,
			 int flush = Z_NO_FLUSH);
	int finish(unsigned char* output, size_t outputCap, size_t* produced);
	void end();
	bool active() const { return active_; }
	static size_t bound(size_t inputLen);

private:
	class Impl;
	Impl* impl_ = nullptr;
	bool active_ = false;
};

#endif
