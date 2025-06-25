#include <iostream>
#include <fstream>
#include <map>
#include <cstring>
#include "uSerial.h"

std::map<std::string, std::string> loadResponses(const std::string& filename) {
    std::map<std::string, std::string> responses;
    std::ifstream file(filename);
    std::string line;

    while (std::getline(file, line)) {
        auto delimiterPos = line.find(':');
        if (delimiterPos != std::string::npos) {
            std::string key = line.substr(0, delimiterPos);
            std::string value = line.substr(delimiterPos + 1);
            responses[key] = value;
        }
    }

    return responses;
}

int main(int argc, char* argv[])
{
    UART::Driver uart;
    std::string port = (argc > 1) ? argv[1] : "/dev/ttyUSB0"; // Default port if no argument is provided
    const uint32_t baudRate = 9600;
    char buffer[UART::Driver::UART_MAX_BUFLENGTH] = {0};

    // Load responses from file
    auto responses = loadResponses("responses.txt");

    // Open the serial port
    if (uart.open(port, baudRate) != UART::Driver::UartStatusCode::SUCCESS) {
        std::cerr << "Failed to open port " << port << std::endl;
        return 1;
    }

    std::cout << "Sending messages from responses.txt on " << port << "..." << std::endl;

    for (const auto& pair : responses) {
        const std::string& message = pair.first;
        const std::string& expectedResponse = pair.second;

        // Send the message
        std::string fullMessage = message + "\n";
        uart.timeout_write(UART::Driver::UART_WRITE_DEFAULT_TIMEOUT, fullMessage.c_str(), fullMessage.length());
        std::cout << "Sent: " << fullMessage << std::endl;

        // Wait for the response
        memset(buffer, 0, sizeof(buffer));
        auto status = uart.timeout_readline(UART::Driver::UART_READ_DEFAULT_TIMEOUT, buffer, sizeof(buffer));
        if (status == UART::Driver::UartStatusCode::SUCCESS) {
            std::string received(buffer);
            std::cout << "Received: " << received << std::endl;

            if (received == expectedResponse) {
                std::cout << "Response matches expected: " << expectedResponse << std::endl;
            } else {
                std::cout << "Unexpected response: " << received << std::endl;
            }
        } else {
            std::cout << "No response received for message: " << message << std::endl;
        }
    }

    uart.close();
    return 0;
}
