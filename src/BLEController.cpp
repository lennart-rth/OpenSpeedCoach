#include "BLEController.h"

#include <bluefruit.h>
#include <SdFat.h>
#include <cctype>
#include <cstring>
#include <stdarg.h>
#include <cstdlib>
#include "Compression.h"
#include "Base64.h"

namespace {
constexpr uint32_t BLE_BOOT_TIMEOUT_MS = 15000;
constexpr size_t BLE_RX_LINE_MAX = 192;
constexpr size_t BLE_TRANSFER_CHUNK = 180;

BLEUart bleuart;

bool textEqualsIgnoreCase(const char* lhs, const char* rhs) {
	while (*lhs && *rhs) {
		if (toupper(static_cast<unsigned char>(*lhs)) != toupper(static_cast<unsigned char>(*rhs))) {
			return false;
		}
		++lhs;
		++rhs;
	}
	return *lhs == '\0' && *rhs == '\0';
}
}

BLEController* BLEController::instance = nullptr;

BLEController::BLEController() {
	instance = this;
}

void BLEController::handleConnectStatic(uint16_t connHandle) {
	if (instance) {
		instance->onConnect(connHandle);
	}
}

void BLEController::handleDisconnectStatic(uint16_t connHandle, uint8_t reason) {
	if (instance) {
		instance->onDisconnect(connHandle, reason);
	}
}

void BLEController::handleAdvStopStatic() {
	if (instance) {
		instance->onAdvStop();
	}
}

void BLEController::beginBootMode() {
	Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
	Bluefruit.begin();
	Bluefruit.setTxPower(4);
	Bluefruit.setName("OpenSpeedCoach");
	Bluefruit.Periph.setConnectCallback(handleConnectStatic);
	Bluefruit.Periph.setDisconnectCallback(handleDisconnectStatic);

	bleuart.begin();
	Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
	Bluefruit.Advertising.addTxPower();
	Bluefruit.Advertising.addService(bleuart);
	Bluefruit.ScanResponse.addName();
	Bluefruit.Advertising.restartOnDisconnect(false);
	Bluefruit.Advertising.setStopCallback(handleAdvStopStatic);
	Bluefruit.Advertising.setInterval(32, 244);
	Bluefruit.Advertising.setFastTimeout(BLE_BOOT_TIMEOUT_MS / 1000);
	Bluefruit.Advertising.start(BLE_BOOT_TIMEOUT_MS / 1000);

	bootActive = true;
	connected = false;
	shutdownRequested = false;
	rxLen = 0;
	rxLine[0] = '\0';
}

void BLEController::poll() {
	if (shutdownRequested) {
		shutdownNow();
		return;
	}

	if (!bootActive || !connected) {
		rxLen = 0;
		return;
	}

	pollInput();
}

void BLEController::shutdown() {
	shutdownRequested = true;
	shutdownNow();
}

void BLEController::shutdownNow() {
	if (!bootActive) {
		shutdownRequested = false;
		return;
	}

	bootActive = false;
	connected = false;
	rxLen = 0;
	rxLine[0] = '\0';

	if (Bluefruit.Advertising.isRunning()) {
		Bluefruit.Advertising.stop();
	}
	sd_softdevice_disable();
	shutdownRequested = false;
}

void BLEController::onConnect(uint16_t connHandle) {
	(void) connHandle;
	connected = true;
}

void BLEController::onDisconnect(uint16_t connHandle, uint8_t reason) {
	(void) connHandle;
	(void) reason;
	connected = false;
	shutdownRequested = true;
}

void BLEController::onAdvStop() {
	if (!connected) {
		shutdownRequested = true;
	}
}

void BLEController::sendLine(const char* text) {
	if (!bootActive || !connected || !bleuart.notifyEnabled()) {
		return;
	}
	size_t len = strlen(text);
	const uint8_t* ptr = reinterpret_cast<const uint8_t*>(text);
	
	// FIX: Loop until every single byte is written, yielding if the buffer is full
	while (len > 0) {
		size_t w = bleuart.write(ptr, len);
		ptr += w;
		len -= w;
		if (len > 0) delay(2); 
	}
	
	uint8_t nl = '\n';
	while (bleuart.write(&nl, 1) == 0) delay(2);
}

void BLEController::sendFormat(const char* fmt, ...) {
	char line[220];
	va_list ap;
	va_start(ap, fmt);
	vsnprintf(line, sizeof(line), fmt, ap);
	va_end(ap);
	sendLine(line);
}

void BLEController::sendHelp() {
	sendLine("OK|HELP|LIST,GET <file>,DELETE <file>");
}

void BLEController::sendFileList() {
	File32 root;
	if (!root.open("/")) {
		sendLine("ERR|LIST|open-root");
		return;
	}

	sendLine("BEGIN|LIST");

	File32 entry;
	root.rewind();
	while (entry.openNext(&root, O_RDONLY)) {
		char name[96];
		if (!entry.getName(name, sizeof(name))) {
			entry.close();
			continue;
		}

		sendFormat("FILE|%s|%lu|%u",
				   name,
				   static_cast<unsigned long>(entry.fileSize()),
				   entry.isDir() ? 1u : 0u);
		entry.close();
	}

	sendLine("END|LIST");
	root.close();
}

void BLEController::sendFile(const char* path) {
	if (!path || !*path) {
		sendLine("ERR|GET|missing-file");
		return;
	}

	File32 file;
	if (!file.open(path, O_RDONLY)) {
		sendFormat("ERR|GET|open|%s", path);
		return;
	}

	if (file.isDir()) {
		file.close();
		sendFormat("ERR|GET|is-dir|%s", path);
		return;
	}

	char name[96];
	if (!file.getName(name, sizeof(name))) {
		strncpy(name, path, sizeof(name) - 1);
		name[sizeof(name) - 1] = '\0';
	}

	const uint32_t totalSize = static_cast<uint32_t>(file.fileSize());
	sendFormat("BEGIN|FILE|%s|%lu", name, static_cast<unsigned long>(totalSize));

	uint8_t buffer[BLE_TRANSFER_CHUNK];
	uint32_t sent = 0;
	while (sent < totalSize) {
		size_t toRead = totalSize - sent;
		if (toRead > sizeof(buffer)) {
			toRead = sizeof(buffer);
		}

		int readCount = file.read(buffer, toRead);
		if (readCount <= 0) {
			sendFormat("ERR|GET|read|%s|%lu", name, static_cast<unsigned long>(sent));
			file.close();
			return;
		}

		// FIX: Safely push uncompressed chunks without dropping bytes
		size_t toWrite = static_cast<size_t>(readCount);
		const uint8_t* ptr = buffer;
		while (toWrite > 0) {
			size_t w = bleuart.write(ptr, toWrite);
			ptr += w;
			toWrite -= w;
			if (toWrite > 0) delay(2);
		}

		sent += static_cast<uint32_t>(readCount);
		delay(1);
	}

	sendFormat("END|FILE|%s|%lu", name, static_cast<unsigned long>(sent));
	file.close();
}

void BLEController::sendCompressedFile(const char* path) {
	if (!path || !*path) {
		sendLine("ERR|GETC|missing-file");
		return;
	}

	File32 file;
	if (!file.open(path, O_RDONLY)) {
		sendFormat("ERR|GETC|open|%s", path);
		return;
	}

	if (file.isDir()) {
		file.close();
		sendFormat("ERR|GETC|is-dir|%s", path);
		return;
	}

	char name[96];
	if (!file.getName(name, sizeof(name))) {
		strncpy(name, path, sizeof(name) - 1);
		name[sizeof(name) - 1] = '\0';
	}

	const uint32_t origSize = static_cast<uint32_t>(file.fileSize());
	StreamingCompressor compressor;
	if (!compressor.begin(Z_BEST_COMPRESSION, MAX_WBITS)) {
		file.close();
		sendFormat("ERR|GETC|init|%s", name);
		return;
	}

	sendFormat("BEGIN|FILE|%s|%lu|COMPRESSED|zlib|0", name, static_cast<unsigned long>(origSize));

	constexpr size_t INPUT_CHUNK = 256;
	constexpr size_t OUTPUT_CHUNK = 512;
	uint8_t inputBuffer[INPUT_CHUNK];
	uint8_t outputBuffer[OUTPUT_CHUNK];

	auto sendBase64Lines = [&](const uint8_t* data, size_t len) {
		constexpr size_t RAW_PER_LINE = 120;
		for (size_t offset = 0; offset < len; offset += RAW_PER_LINE) {
			size_t slice = len - offset;
			if (slice > RAW_PER_LINE) slice = RAW_PER_LINE;
			char* encoded = base64_encode_alloc(data + offset, slice);
			if (!encoded) {
				return false;
			}
			sendLine(encoded);
			free(encoded);
		}
		return true;
	};

	bool transferFailed = false;
	uint32_t totalRead = 0;   // NEW: Track bytes read from SD card
	int lastProgress = -1;    // NEW: Track last percentage sent

	while (!transferFailed) {
		int readCount = file.read(inputBuffer, sizeof(inputBuffer));
		if (readCount < 0) {
			sendFormat("ERR|GETC|read|%s|%d", name, readCount);
			transferFailed = true;
			break;
		}
		if (readCount == 0) {
			break;
		}

		// FIX: Calculate uncompressed reading progress and send it to the UI
		totalRead += static_cast<uint32_t>(readCount);
		int currentProgress = (totalRead * 100) / origSize;
		if (currentProgress >= lastProgress + 2) { // Update the UI every 2%
			sendFormat("PROG|%d", currentProgress);
			lastProgress = currentProgress;
		}

		size_t consumedTotal = 0;
		while (consumedTotal < static_cast<size_t>(readCount)) {
			size_t produced = 0;
			size_t consumedThis = 0;
			const int status = compressor.feed(inputBuffer + consumedTotal,
									   static_cast<size_t>(readCount) - consumedTotal,
									   outputBuffer,
									   sizeof(outputBuffer),
									   &consumedThis,
									   &produced,
									   Z_NO_FLUSH);
			consumedTotal += consumedThis;

			if (produced > 0 && !sendBase64Lines(outputBuffer, produced)) {
				sendFormat("ERR|GETC|tx|%s", name);
				transferFailed = true;
				break;
			}

			if ((status != Z_OK) && (status != Z_BUF_ERROR)) {
				sendFormat("ERR|GETC|compress|%s|%d", name, status);
				transferFailed = true;
				break;
			}

			if ((consumedThis == 0) && (produced == 0)) {
				// Defensive break to avoid a stall if the compressor reports no progress.
				break;
			}
		}
		delay(1);
	}

	if (!transferFailed) {
		for (;;) {
			size_t produced = 0;
			const int status = compressor.finish(outputBuffer, sizeof(outputBuffer), &produced);
			if (produced > 0 && !sendBase64Lines(outputBuffer, produced)) {
				sendFormat("ERR|GETC|tx|%s", name);
				transferFailed = true;
				break;
			}
			if (status == Z_STREAM_END) {
				break;
			}
			if ((status != Z_OK) && (status != Z_BUF_ERROR)) {
				sendFormat("ERR|GETC|finish|%s|%d", name, status);
				transferFailed = true;
				break;
			}
		}
	}

	compressor.end();
	file.close();

	if (transferFailed) {
		return;
	}

	sendFormat("END|FILE|%s|%lu", name, static_cast<unsigned long>(origSize));
}

void BLEController::deleteFile(const char* path) {
	if (!path || !*path) {
		sendLine("ERR|DELETE|missing-file");
		return;
	}

	File32 file;
	if (!file.open(path, O_RDONLY)) {
		sendFormat("ERR|DELETE|open|%s", path);
		return;
	}

	if (!file.remove()) {
		file.close();
		sendFormat("ERR|DELETE|remove|%s", path);
		return;
	}

	file.close();
	sendFormat("OK|DELETE|%s", path);
}

void BLEController::handleCommand(char* line) {
	while (*line == ' ' || *line == '\t') {
		++line;
	}
	if (*line == '\0') {
		return;
	}

	char* arg = line;
	while (*arg && *arg != ' ' && *arg != '\t') {
		++arg;
	}
	if (*arg) {
		*arg++ = '\0';
		while (*arg == ' ' || *arg == '\t') {
			++arg;
		}
	}

	if (textEqualsIgnoreCase(line, "HELP")) {
		sendHelp();
	} else if (textEqualsIgnoreCase(line, "LIST")) {
		sendFileList();
	} else if (textEqualsIgnoreCase(line, "GET")) {
		sendFile(arg);
	} else if (textEqualsIgnoreCase(line, "GETC")) {
		// Compressed GET (base64'd compressed file)
		sendCompressedFile(arg);
	} else if (textEqualsIgnoreCase(line, "DELETE")) {
		deleteFile(arg);
	} else {
		sendFormat("ERR|UNKNOWN|%s", line);
	}
}

void BLEController::pollInput() {
	while (bleuart.available() > 0) {
		char c = static_cast<char>(bleuart.read());
		if (c == '\r') {
			continue;
		}
		if (c == '\n') {
			rxLine[rxLen] = '\0';
			handleCommand(rxLine);
			rxLen = 0;
			continue;
		}
		if (rxLen + 1 < BLE_RX_LINE_MAX) {
			rxLine[rxLen++] = c;
		}
	}
}