
#include <iostream>
#include <fstream>
#include <string>
#include <ctime>
#include <boost/asio.hpp>

using namespace boost::asio;

// get the current time in "HH_MM_SS"
std::string get_timestamp() {
    std::time_t now = std::time(nullptr);
    char buf[20];
    std::strftime(buf, sizeof(buf), "%H_%M_%S", std::localtime(&now));
    return buf;
}

// read data from the serial port and save it to a file
void receive_data(const std::string& port, unsigned int baud_rate) {
    io_service io;
    serial_port serial(io, port);
    serial.set_option(serial_port_base::baud_rate(baud_rate));

    // wait for data to be available on the serial port
    while (true) {
        boost::system::error_code ec;
        if (serial.available(ec) > 0) {
            break;
        }
    }

    // read the header to get the file size
    boost::asio::streambuf header;
    boost::asio::read_until(serial, header, "\n");
    std::istream header_stream(&header);
    unsigned long file_size;
    header_stream >> file_size;

    std::string timestamp = get_timestamp();
    std::string filename = "BMS_data_" + timestamp + ".csv";
    std::ofstream outfile(filename);

    if (!outfile.is_open()) {
        std::cerr << "Failed to open output file: " << filename << std::endl;
        return;
    }

    char c;
    unsigned long bytes_received = 0;

    while (bytes_received < file_size) {
        boost::system::error_code ec;
        read(serial, buffer(&c, 1), ec);
        if (ec) {
            std::cerr << "Error: " << ec.message() << std::endl;
            break;
        }
        outfile.put(c);    // write character to file
        //std::cout.put(c);  // print character to console
        bytes_received++;
    }

    outfile.close();
    std::cout << "\nReceived " << bytes_received << " bytes. Saved to " << filename << std::endl;
}

int main() {
    //serial port name depends on computer running program
    std::string serial_port = "/dev/ttyACM0"; 
    unsigned int baud_rate = 9600;

    try {
        receive_data(serial_port, baud_rate);
    } catch (boost::system::system_error& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
