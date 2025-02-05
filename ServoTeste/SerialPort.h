#ifndef SERIALPORT_H
#define SERIALPORT_H

#include <string>
#include <stdexcept>
#include <sstream>
#include <vector>

#ifdef _WIN32
#include <windows.h>
#else
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#endif

/**
 * @class SerialPort
 * @brief Cross-platform class for managing serial port communication on Windows and Linux.
 */
class SerialPort {
private:
#ifdef _WIN32
    HANDLE hSerial;            ///< Handle for the serial port (Windows).
    COMMTIMEOUTS timeouts;     ///< Structure for timeouts (Windows).
    DCB dcbSerialParams;       ///< Structure for serial port parameters (Windows).
#else
    int serialFd;              ///< File descriptor for the serial port (Linux).
    struct termios tty;        ///< Structure for serial port configuration (Linux).
#endif

public:
    // --- Constants for baud rates ---
    static const int baud110;
    static const int baud300;
    static const int baud600;
    static const int baud1200;
    static const int baud2400;
    static const int baud4800;
    static const int baud9600;
    static const int baud14400;
    static const int baud19200;
    static const int baud38400;
    static const int baud57600;
    static const int baud115200;

    // --- Constants for parity ---
    static const int parityNone;
    static const int parityOdd;
    static const int parityEven;

    /**
     * @brief SerialPort class constructor.
     */
    SerialPort();

    /**
     * @brief SerialPort class destructor.
     */
    ~SerialPort();

    /**
     * @brief Opens the specified serial port.
     * @param portName Serial port name (e.g., "COM1" on Windows or "/dev/ttyS0" on Linux).
     * @throw std::runtime_error If an error occurs while opening the port.
     */
    void openPort(const std::string& portName);

    /**
     * @brief Configures the serial port parameters.
     * @param baudRate Data transmission rate. Use one of the constants defined in the class.
     * @param parity Communication parity. Use one of the constants defined in the class.
     * @throw std::runtime_error If an error occurs during configuration.
     */
    void configurePort(int baudRate, int parity);

    /**
     * @brief Sends a string of data through the serial port.
     * @param data Data to be sent as a string.
     * @throw std::runtime_error If an error occurs while sending data.
     */
    void sendData(const std::string& data);

    /**
     * @brief Receives data from the serial port as a string.
     * @param bufferSize Maximum size of data to read.
     * @return Received data as a string.
     * @throw std::runtime_error If an error occurs while reading data.
     */
    std::string receiveData(size_t bufferSize);

    /**
     * @brief Checks if the serial port is currently connected/open.
     * @return True if the serial port is open, false otherwise.
     */
    bool isConnected() const;

    /**
     * @brief Checks the number of bytes available for reading on the serial port.
     * @return The number of bytes available to read.
     * @throw std::runtime_error If an error occurs while checking the available data.
     */
    size_t availableData() const;

    /**
     * @brief Clears the input buffer of the serial port.
     * @throw std::runtime_error If an error occurs while clearing the buffer.
     */
    void clearBuffer();

    /**
     * @brief Closes the serial port if it is open.
     */
    void closePort();

    /**
     * @brief Overloads the output operator for sending data.
     * @tparam T Data type.
     * @param data Data to send.
     * @return Reference to the SerialPort object.
     * @throw std::runtime_error If an error occurs while sending data.
     */
    template <typename T>
    SerialPort& operator<<(const T& data) {
        std::stringstream oss;
        oss << data;
        sendData(oss.str());
        return *this;
    }

    /**
     * @brief Overloads the input operator for receiving data.
     * @tparam T Data type.
     * @param data Reference to the variable to store received data.
     * @return Reference to the SerialPort object.
     * @throw std::runtime_error If an error occurs while reading data.
     */
    template <typename T>
    SerialPort& operator>>(T& data) {
        std::string received = receiveData(1024); // Default buffer size
        std::istringstream iss(received);
        iss >> data;
        if (iss.fail()) {
            throw std::runtime_error("Error parsing received data.");
        }
        return *this;
    }
};

#endif // SERIALPORT_H
