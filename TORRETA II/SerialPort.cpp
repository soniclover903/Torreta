#include "SerialPort.h"
#include <sstream>
#include <cstring> // For strerror on Linux
#include <stdexcept>

// --- Baud rate constants ---
const int SerialPort::baud110 =
#ifdef _WIN32
    CBR_110;
#else
    B110;
#endif

const int SerialPort::baud300 =
#ifdef _WIN32
    CBR_300;
#else
    B300;
#endif

const int SerialPort::baud600 =
#ifdef _WIN32
    CBR_600;
#else
    B600;
#endif

const int SerialPort::baud1200 =
#ifdef _WIN32
    CBR_1200;
#else
    B1200;
#endif

const int SerialPort::baud2400 =
#ifdef _WIN32
    CBR_2400;
#else
    B2400;
#endif

const int SerialPort::baud4800 =
#ifdef _WIN32
    CBR_4800;
#else
    B4800;
#endif

const int SerialPort::baud9600 =
#ifdef _WIN32
    CBR_9600;
#else
    B9600;
#endif

const int SerialPort::baud14400 =
#ifdef _WIN32
    CBR_14400;
#else
    B19200; // Approximation for Linux
#endif

const int SerialPort::baud19200 =
#ifdef _WIN32
    CBR_19200;
#else
    B19200;
#endif

const int SerialPort::baud38400 =
#ifdef _WIN32
    CBR_38400;
#else
    B38400;
#endif

const int SerialPort::baud57600 =
#ifdef _WIN32
    CBR_57600;
#else
    B57600;
#endif

const int SerialPort::baud115200 =
#ifdef _WIN32
    CBR_115200;
#else
    B115200;
#endif

// --- Parity constants ---
const int SerialPort::parityNone =
#ifdef _WIN32
    NOPARITY;
#else
    0; // No parity for Linux
#endif

const int SerialPort::parityOdd =
#ifdef _WIN32
    ODDPARITY;
#else
    1; // Odd parity flag for Linux
#endif

const int SerialPort::parityEven =
#ifdef _WIN32
    EVENPARITY;
#else
    2; // Even parity flag for Linux
#endif

#ifdef _WIN32
// Windows implementation
SerialPort::SerialPort() : hSerial(INVALID_HANDLE_VALUE) {}

SerialPort::~SerialPort() {
    closePort();
}

void SerialPort::openPort(const std::string& portName) {
    hSerial = CreateFile(portName.c_str(),
                         GENERIC_READ | GENERIC_WRITE,
                         0,
                         NULL,
                         OPEN_EXISTING,
                         0,
                         NULL);
    if (hSerial == INVALID_HANDLE_VALUE) {
        throw std::runtime_error("Error opening serial port: " + std::to_string(GetLastError()));
    }
}

void SerialPort::configurePort(int baudRate, int parity) {
    dcbSerialParams = { 0 };
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);

    if (!GetCommState(hSerial, &dcbSerialParams)) {
        throw std::runtime_error("Error getting serial port state.");
    }

    dcbSerialParams.BaudRate = baudRate;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity = parity;

    if (!SetCommState(hSerial, &dcbSerialParams)) {
        throw std::runtime_error("Error setting serial port parameters.");
    }

    timeouts = { 0 };
    timeouts.ReadIntervalTimeout = 50;
    timeouts.ReadTotalTimeoutConstant = 50;
    timeouts.WriteTotalTimeoutConstant = 50;

    if (!SetCommTimeouts(hSerial, &timeouts)) {
        throw std::runtime_error("Error setting serial port timeouts.");
    }
}

void SerialPort::sendData(const std::string& data) {
    DWORD bytesWritten;
    if (!WriteFile(hSerial, data.c_str(), data.length(), &bytesWritten, NULL)) {
        throw std::runtime_error("Error sending data through the serial port.");
    }
}

std::string SerialPort::receiveData(size_t bufferSize) {
    char* buffer = new char[bufferSize]();
    DWORD bytesRead;
    if (ReadFile(hSerial, buffer, bufferSize, &bytesRead, NULL)) {
        std::string result(buffer, bytesRead);
        delete[] buffer;
        return result;
    } else {
        delete[] buffer;
        throw std::runtime_error("Error reading data from the serial port.");
    }
}

void SerialPort::closePort() {
    if (hSerial != INVALID_HANDLE_VALUE) {
        CloseHandle(hSerial);
        hSerial = INVALID_HANDLE_VALUE;
    }
}

bool SerialPort::isConnected() const {
    return hSerial != INVALID_HANDLE_VALUE;
}

size_t SerialPort::availableData() const {
    if (hSerial == INVALID_HANDLE_VALUE) {
        throw std::runtime_error("Serial port not open.");
    }

    COMSTAT status;
    DWORD errors;
    if (!ClearCommError(hSerial, &errors, &status)) {
        throw std::runtime_error("Error checking serial port status: " + std::to_string(GetLastError()));
    }
    return status.cbInQue; // Number of bytes in the input buffer
}

void SerialPort::clearBuffer() {
    if (hSerial == INVALID_HANDLE_VALUE) {
        throw std::runtime_error("Serial port not open.");
    }

    if (!PurgeComm(hSerial, PURGE_RXCLEAR)) {
        throw std::runtime_error("Error clearing serial port buffer: " + std::to_string(GetLastError()));
    }
}

#else
// Linux implementation
SerialPort::SerialPort() : serialFd(-1) {}

SerialPort::~SerialPort() {
    closePort();
}

void SerialPort::openPort(const std::string& portName) {
    serialFd = open(portName.c_str(), O_RDWR | O_NOCTTY);
    if (serialFd == -1) {
        throw std::runtime_error("Error opening serial port: " + std::string(strerror(errno)));
    }
}

void SerialPort::configurePort(int baudRate, int parity) {
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(serialFd, &tty) != 0) {
        throw std::runtime_error("Error getting serial port attributes: " + std::string(strerror(errno)));
    }

    cfsetospeed(&tty, baudRate);
    cfsetispeed(&tty, baudRate);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit characters
    tty.c_iflag = 0;                            // No special input processing
    tty.c_oflag = 0;                            // No special output processing
    tty.c_lflag = 0;                            // No local flags
    tty.c_cflag |= (CLOCAL | CREAD);            // Enable receiver and set local mode

    if (parity == 1) {
        tty.c_cflag |= PARENB; // Enable parity
        tty.c_cflag |= PARODD; // Odd parity
    } else if (parity == 2) {
        tty.c_cflag |= PARENB; // Enable parity
        tty.c_cflag &= ~PARODD; // Even parity
    } else {
        tty.c_cflag &= ~PARENB; // No parity
    }

    if (tcsetattr(serialFd, TCSANOW, &tty) != 0) {
        throw std::runtime_error("Error setting serial port attributes: " + std::string(strerror(errno)));
    }
}

void SerialPort::sendData(const std::string& data) {
    ssize_t bytesWritten = write(serialFd, data.c_str(), data.size());
    if (bytesWritten == -1) {
        throw std::runtime_error("Error sending data through the serial port: " + std::string(strerror(errno)));
    }
}

std::string SerialPort::receiveData(size_t bufferSize) {
    char* buffer = new char[bufferSize]();
    ssize_t bytesRead = read(serialFd, buffer, bufferSize);
    if (bytesRead == -1) {
        delete[] buffer;
        throw std::runtime_error("Error reading data from the serial port: " + std::string(strerror(errno)));
    }
    std::string result(buffer, bytesRead);
    delete[] buffer;
    return result;
}

void SerialPort::closePort() {
    if (serialFd != -1) {
        close(serialFd);
        serialFd = -1;
    }
}

bool SerialPort::isConnected() const {
    return serialFd != -1;
}

size_t SerialPort::availableData() const {
    if (serialFd == -1) {
        throw std::runtime_error("Serial port not open.");
    }

    int bytesAvailable;
    if (ioctl(serialFd, FIONREAD, &bytesAvailable) == -1) {
        throw std::runtime_error("Error checking available data: " + std::string(strerror(errno)));
    }
    return static_cast<size_t>(bytesAvailable);
}

void SerialPort::clearBuffer() {
    if (serialFd == -1) {
        throw std::runtime_error("Serial port not open.");
    }

    tcflush(serialFd, TCIFLUSH); // Clears the input buffer
}

#endif
