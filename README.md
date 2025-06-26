
# uSerial

**uSerial** is a UART (Universal Asynchronous Receiver/Transmitter) serial driver designed to facilitate asynchronous serial communication between a microcontroller (or processor) and external devices such as sensors, modems, or other microcontrollers. It handles the transmission and reception of data over UART hardware interfaces.

## Supported Platforms
- Linux
- Windows

## Usage Overview
Include the `uSerial.h` header file in your project and use the provided methods to configure and manage UART communication.

## Available Methods
- `UartStatusCode open(const std::string& strDevice, uint32_t u32Speed);`
- `UartStatusCode close();`
- `UartStatusCode purge(bool bInput, bool bOutput);`

- `UartStatusCode timeout_read(uint32_t u32ReadTimeout, char *pBuffer, size_t szSizeToRead);`
- `UartStatusCode timeout_read(uint32_t u32ReadTimeout, char *pBuffer, size_t szSizeToRead, size_t* pBytesRead);`
- `UartStatusCode timeout_readline(uint32_t u32ReadTimeout, char *pBuffer, size_t szBufferSize);`

- `UartStatusCode timeout_wait_for_token_buffer(uint32_t u32ReadTimeout, const char *pstrToken, uint32_t u32TokenLength);`

- `UartStatusCode timeout_write(uint32_t u32WriteTimeout, const char *pBuffer, size_t szSizeToWrite);`
- `UartStatusCode timeout_wait_for_token(uint32_t u32ReadTimeout, const char *pstrToken);`

### Note:
This is a work in progress, new methods are intended to be added, etc.