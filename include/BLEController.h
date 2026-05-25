#ifndef BLECONTROLLER_H
#define BLECONTROLLER_H

#include <Arduino.h>

class BLEController {
public:
	BLEController();

	void beginBootMode();
	void poll();
	void shutdown();

	bool isBootActive() const { return bootActive; }
	bool isConnected() const { return connected; }

private:
	static BLEController* instance;

	bool bootActive = false;
	bool connected = false;
	bool shutdownRequested = false;
	char rxLine[192] = {};
	size_t rxLen = 0;

	void sendLine(const char* text);
	void sendFormat(const char* fmt, ...);
	void shutdownNow();
	void onConnect(uint16_t connHandle);
	void onDisconnect(uint16_t connHandle, uint8_t reason);
	void onAdvStop();
	void sendHelp();
	void sendFileList();
	void sendFile(const char* path);
	void sendCompressedFile(const char* path);
	void deleteFile(const char* path);
	void handleCommand(char* line);
	void pollInput();

	static void handleConnectStatic(uint16_t connHandle);
	static void handleDisconnectStatic(uint16_t connHandle, uint8_t reason);
	static void handleAdvStopStatic();
};

#endif