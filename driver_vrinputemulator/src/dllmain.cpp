#include "logging.h"

const char* logConfigFileName = "logging.conf";

const char* logConfigDefault =
"* GLOBAL:\n"
"	FORMAT = \"[%level] %datetime{%Y-%M-%d %H:%m:%s}: %msg\"\n"
"	FILENAME = \"driver_vrinputemulator.log\"\n"
"	ENABLED = true\n"
"	TO_FILE = true\n"
"	TO_STANDARD_OUTPUT = true\n"
"	MAX_LOG_FILE_SIZE = 2097152 ## 2MB\n"
"* TRACE:\n"
"	ENABLED = true\n"
"* DEBUG:\n"
"	ENABLED = false\n";

WSADATA wsaData;
bool wsaInitialized = false;

INITIALIZE_EASYLOGGINGPP

void init_logging() {
	el::Loggers::addFlag(el::LoggingFlag::DisableApplicationAbortOnFatalLog);
	el::Configurations conf(logConfigFileName);
	conf.parseFromText(logConfigDefault);
	conf.parseFromFile(logConfigFileName);
	conf.setRemainingToDefault();
	el::Loggers::reconfigureAllLoggers(conf);
}

void init_socketLibrary() {
	if (wsaInitialized)
		return;
	int err = WSAStartup(MAKEWORD(2, 2), &wsaData);
	if (err != 0) {
		LOG(ERROR) << "Error while initializing Window sockets library: Error code " << err;
	}
	if (LOBYTE(wsaData.wVersion) != 2 || HIBYTE(wsaData.wVersion) != 2) {
		LOG(ERROR) << "Could not find a usable version of Window sockets library: Version " << LOBYTE(wsaData.wVersion) << "." << HIBYTE(wsaData.wVersion) << " found";
		WSACleanup();
	} else {
		wsaInitialized = true;
		LOG(INFO) << "Window sockets library initialized";
	}
}

void shutdown_socketLibrary() {
	if (!wsaInitialized)
		return;
	int err = WSACleanup();
	if (err != 0) {
		LOG(ERROR) << "Error while finalizing Window sockets library: Error code " << err;
	}
}

BOOL APIENTRY DllMain( HMODULE hModule,
                       DWORD  ul_reason_for_call,
                       LPVOID lpReserved
					 ) {
	switch (ul_reason_for_call) {
		case DLL_PROCESS_ATTACH:
			init_logging();
			init_socketLibrary();
			LOG(INFO) << "VRInputEmulator dll loaded...";
			break;
		case DLL_THREAD_ATTACH:
		case DLL_THREAD_DETACH:
			break;
		case DLL_PROCESS_DETACH:
			shutdown_socketLibrary();
			break;
	}
	return TRUE;
}

