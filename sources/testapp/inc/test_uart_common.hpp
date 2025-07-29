#ifndef TEST_UART_COMMON_HPP
#define TEST_UART_COMMON_HPP

#include <iostream>
#include <fstream>
#include <string>
#include <map>

#define TEST_FILENAME               "responses.txt"
#define TEST_HEXDUMP_LINE_LEN       16U

bool fileExistsAndNotEmpty(const std::string& filename);
std::map<std::string, std::string> loadResponses(const std::string& filename);


#endif //TEST_UART_COMMON_HPP