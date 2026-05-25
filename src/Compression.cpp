#include "Compression.h"

#include <cstring>
#include <zlib.h>

class StreamingCompressor::Impl {
public:
	z_stream stream{};
};

StreamingCompressor::StreamingCompressor() = default;

StreamingCompressor::~StreamingCompressor() {
	end();
}

bool StreamingCompressor::begin(int level, int windowBits) {
	end();
	impl_ = new Impl();
	std::memset(&impl_->stream, 0, sizeof(impl_->stream));
    
	// FIX: Lower windowBits to 11 and memLevel to 4 to prevent Out-Of-Memory crashes.
	const int status = deflateInit2(&impl_->stream, level, Z_DEFLATED, 11, 4, Z_DEFAULT_STRATEGY);
    
	active_ = (status == Z_OK);
	if (!active_) {
		delete impl_;
		impl_ = nullptr;
	}
	return active_;
}

int StreamingCompressor::feed(const unsigned char* input,
						  size_t inputLen,
						  unsigned char* output,
						  size_t outputCap,
						  size_t* consumed,
						  size_t* produced,
						  int flush) {
	if (!active_ || !impl_ || !output || !consumed || !produced) {
		return Z_STREAM_ERROR;
	}

	impl_->stream.next_in = const_cast<unsigned char*>(input);
	impl_->stream.avail_in = static_cast<unsigned int>(inputLen);
	impl_->stream.next_out = output;
	impl_->stream.avail_out = static_cast<unsigned int>(outputCap);

	const int status = deflate(&impl_->stream, flush);
	*consumed = inputLen - impl_->stream.avail_in;
	*produced = outputCap - impl_->stream.avail_out;
	return status;
}

int StreamingCompressor::finish(unsigned char* output, size_t outputCap, size_t* produced) {
	size_t consumed = 0;
	return feed(nullptr, 0, output, outputCap, &consumed, produced, Z_FINISH);
}

void StreamingCompressor::end() {
	if (active_ && impl_) {
		deflateEnd(&impl_->stream);
		active_ = false;
	}
	delete impl_;
	impl_ = nullptr;
	active_ = false;
}

size_t StreamingCompressor::bound(size_t inputLen) {
	return inputLen + (inputLen / 16) + 64;
}
