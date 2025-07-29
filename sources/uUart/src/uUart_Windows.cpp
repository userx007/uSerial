#include "uUart.hpp"
#include "uLogger.hpp"

#include <windows.h>
#include <io.h>
#include <fcntl.h>
#include <errno.h>
#include <cstring>


#define LT_HDR     "UART_DRIVER:"
#define LOG_HDR    LOG_STRING(LT_HDR)



UART::Status UART::open(const std::string& strDevice, uint32_t u32Speed)
{
    if (strDevice.empty() || u32Speed == 0) {
        LOG_PRINT(LOG_ERROR, LOG_HDR; LOG_STRING("Invalid parameter(s):");
                  LOG_STRING(strDevice.c_str());
                  LOG_STRING("Baudrate:"); LOG_UINT32(u32Speed));
        return Status::INVALID_PARAM;
    }

#ifdef _MSC_VER
    int openFlags = _O_RDWR | _O_BINARY;
    int shareMode = _SH_DENYRW; // Deny other processes to access the file
    int fileHandle;

    errno_t err = _sopen_s(&fileHandle, strDevice.c_str(), openFlags, shareMode, _S_IREAD | _S_IWRITE);
    if (err == 0) {
        m_iHandle = fileHandle;
    } else {
        m_iHandle = -1;
    }
#else
    int openFlags = O_RDWR | O_NOINHERIT | O_BINARY;
    m_iHandle = _open(strDevice.c_str(), openFlags);
#endif


    if (m_iHandle < 0) {
        int errnoRet = errno;
        LOG_PRINT(LOG_ERROR, LOG_HDR;
                  LOG_STRING("Failed to open ["); LOG_STRING(strDevice.c_str());
                  LOG_UINT32(u32Speed); LOG_STRING("] errno:"); LOG_INT(errnoRet));
        return Status::PORT_ACCESS;
    }

    UART::Status result = setup(u32Speed);
    if (result != Status::SUCCESS) {
        LOG_PRINT(LOG_ERROR, LOG_HDR;
                  LOG_STRING("Failed to configure ["); LOG_STRING(strDevice.c_str());
                  LOG_UINT32(u32Speed); LOG_INT(m_iHandle);
                  LOG_STRING("] Error:"); LOG_INT(result));
        _close(m_iHandle);
        m_iHandle = -1;
        return Status::PORT_ACCESS;
    }

    LOG_PRINT(LOG_DEBUG, LOG_HDR;
              LOG_STRING("UART ["); LOG_STRING(strDevice.c_str());
              LOG_UINT32(u32Speed); LOG_STRING("] opened, handle:");
              LOG_INT(m_iHandle));

    return Status::SUCCESS;
}



UART::Status UART::close()
{
    if (m_iHandle >= 0) {
        _close(m_iHandle);
        LOG_PRINT(LOG_DEBUG, LOG_HDR; LOG_STRING("UART closed, handle:"); LOG_INT(m_iHandle));
        m_iHandle = -1;
    }
    return Status::SUCCESS;
}



UART::Status UART::purge(bool bInput, bool bOutput)  const
{
    HANDLE hCom = (HANDLE)_get_osfhandle(m_iHandle);
    DWORD purgeOptions = 0;
    if (bInput) purgeOptions |= PURGE_RXCLEAR;
    if (bOutput) purgeOptions |= PURGE_TXCLEAR;

    if (!PurgeComm(hCom, purgeOptions)) {
        LOG_PRINT(LOG_ERROR, LOG_HDR; LOG_STRING("PurgeComm() failed for handle:"); LOG_INT(m_iHandle));
        return Status::FLUSH_FAILED;
    }

    return Status::SUCCESS;
}



UART::Status UART::timeout_read(uint32_t u32ReadTimeout, std::span<uint8_t> buffer, size_t& szBytesRead) const
{
    if (buffer.empty()) {
        LOG_PRINT(LOG_ERROR, LOG_HDR; LOG_STRING("timeout_read: invalid parameter"));
        return Status::INVALID_PARAM;
    }

    HANDLE hCom = (HANDLE)_get_osfhandle(m_iHandle);
    if (hCom == INVALID_HANDLE_VALUE) {
        LOG_PRINT(LOG_ERROR, LOG_HDR; LOG_STRING("Invalid handle from _get_osfhandle"));
        return Status::PORT_ACCESS;
    }

    COMMTIMEOUTS originalTimeouts;
    if (!GetCommTimeouts(hCom, &originalTimeouts)) {
        LOG_PRINT(LOG_ERROR, LOG_HDR; LOG_STRING("Failed to get original COMMTIMEOUTS"));
        return Status::PORT_ACCESS;
    }

    COMMTIMEOUTS newTimeouts = originalTimeouts;
    newTimeouts.ReadIntervalTimeout = 0;
    newTimeouts.ReadTotalTimeoutMultiplier = 0;
    newTimeouts.ReadTotalTimeoutConstant = u32ReadTimeout;

    if (!SetCommTimeouts(hCom, &newTimeouts)) {
        LOG_PRINT(LOG_ERROR, LOG_HDR; LOG_STRING("Failed to set COMMTIMEOUTS"));
        return Status::PORT_ACCESS;
    }

    size_t szTotalBytesRead = 0;
    while (szTotalBytesRead < buffer.size()) {
        DWORD dwBytesToRead = static_cast<DWORD>(buffer.size() - szTotalBytesRead);
        int iBytesRead = _read(m_iHandle, buffer.data() + szTotalBytesRead, dwBytesToRead);

        if (iBytesRead < 0) {
            int err = errno;
            LOG_PRINT(LOG_ERROR, LOG_HDR; LOG_STRING("_read() failed"); LOG_INT(err));
            SetCommTimeouts(hCom, &originalTimeouts);
            return Status::READ_ERROR;
        } else if (iBytesRead == 0) {
            SetCommTimeouts(hCom, &originalTimeouts);
            return Status::READ_TIMEOUT;
        }

        szTotalBytesRead += iBytesRead;
    }

    SetCommTimeouts(hCom, &originalTimeouts);
    szBytesRead = szTotalBytesRead;
    return Status::SUCCESS;
}



UART::Status UART::timeout_write(uint32_t u32WriteTimeout, std::span<const uint8_t> buffer) const
{
    if (buffer.empty()) {
        LOG_PRINT(LOG_ERROR, LOG_HDR; LOG_STRING("Invalid parameter: buffer.empty()"));
        return Status::INVALID_PARAM;
    }

    if (buffer.size() == 0) return Status::SUCCESS;

    HANDLE hCom = (HANDLE)_get_osfhandle(m_iHandle);
    if (hCom == INVALID_HANDLE_VALUE) {
        LOG_PRINT(LOG_ERROR, LOG_HDR; LOG_STRING("Invalid handle from _get_osfhandle"));
        return Status::PORT_ACCESS;
    }

    COMMTIMEOUTS originalTimeouts;
    if (!GetCommTimeouts(hCom, &originalTimeouts)) {
        LOG_PRINT(LOG_ERROR, LOG_HDR;  LOG_STRING("Failed to get original COMMTIMEOUTS"));
        return Status::PORT_ACCESS;
    }

    COMMTIMEOUTS newTimeouts = originalTimeouts;
    newTimeouts.WriteTotalTimeoutMultiplier = 0;
    newTimeouts.WriteTotalTimeoutConstant = u32WriteTimeout;

    if (!SetCommTimeouts(hCom, &newTimeouts)) {
        LOG_PRINT(LOG_ERROR, LOG_HDR; LOG_STRING("Failed to set COMMTIMEOUTS"));
        return Status::PORT_ACCESS;
    }

    size_t szTotalBytesWritten = 0;
    while (szTotalBytesWritten < buffer.size()) {
        int iBytesWritten = _write(m_iHandle, buffer.data() + szTotalBytesWritten, static_cast<unsigned int>(buffer.size() - szTotalBytesWritten));
        if (iBytesWritten <= 0) {
            SetCommTimeouts(hCom, &originalTimeouts);
            return (iBytesWritten == 0) ? Status::WRITE_TIMEOUT : Status::WRITE_ERROR;
        }
        szTotalBytesWritten += iBytesWritten;
    }

    SetCommTimeouts(hCom, &originalTimeouts);
    purge(true, true);
    return Status::SUCCESS;
}



UART::Status UART::setup(uint32_t u32Speed) const
{
    HANDLE hCom = (HANDLE)_get_osfhandle(m_iHandle);
    DCB dcb;
    ZeroMemory(&dcb, sizeof(DCB));
    dcb.DCBlength = sizeof(DCB);

    if (!GetCommState(hCom, &dcb)) {
        LOG_PRINT(LOG_ERROR, LOG_HDR; LOG_STRING("GetCommState() failed for handle:"); LOG_INT(m_iHandle));
        return Status::PORT_ACCESS;
    }

    dcb.BaudRate = u32Speed;
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    dcb.fBinary = TRUE;
    dcb.fInX = FALSE;
    dcb.fOutX = FALSE;
    dcb.fRtsControl = RTS_CONTROL_DISABLE;
    dcb.fDtrControl = DTR_CONTROL_DISABLE;
    dcb.fOutxCtsFlow = FALSE;
    dcb.fOutxDsrFlow = FALSE;
    dcb.fNull = FALSE;
    dcb.fErrorChar = FALSE;
    dcb.fAbortOnError = FALSE;

    if (!SetCommState(hCom, &dcb)) {
        LOG_PRINT(LOG_ERROR, LOG_HDR; LOG_STRING("SetCommState() failed for handle:"); LOG_INT(m_iHandle));
        return Status::PORT_ACCESS;
    }

    purge(true, true);
    return Status::SUCCESS;
}



uint32_t UART::getBaud(uint32_t u32Speed) const
{
    // On Windows, baud rates are usually passed directly as integers.
    // You can validate or log unsupported values here if needed.
    return u32Speed;
}
