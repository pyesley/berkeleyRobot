#include "tcp_server.hpp"

#include <cstring>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <poll.h>
#include <fcntl.h>
#include <iostream>
#include <sstream>
#include <regex>

TCPServer::TCPServer() : server_fd_(-1), client_fd_(-1), port_(0) {}

TCPServer::~TCPServer() {
    stop();
}

bool TCPServer::start(int port) {
    port_ = port;

    // Create socket
    server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd_ < 0) {
        std::cerr << "Error creating TCP socket: " << strerror(errno) << std::endl;
        return false;
    }

    // Allow address reuse
    int opt = 1;
    setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    // Set non-blocking
    int flags = fcntl(server_fd_, F_GETFL, 0);
    fcntl(server_fd_, F_SETFL, flags | O_NONBLOCK);

    // Bind
    struct sockaddr_in addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = inet_addr("127.0.0.1");
    addr.sin_port = htons(port);

    if (bind(server_fd_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        std::cerr << "Error binding TCP socket: " << strerror(errno) << std::endl;
        close(server_fd_);
        server_fd_ = -1;
        return false;
    }

    // Listen
    if (listen(server_fd_, 1) < 0) {
        std::cerr << "Error listening on TCP socket: " << strerror(errno) << std::endl;
        close(server_fd_);
        server_fd_ = -1;
        return false;
    }

    std::cout << "TCP server listening on localhost:" << port << std::endl;
    return true;
}

void TCPServer::stop() {
    closeClient();
    if (server_fd_ >= 0) {
        close(server_fd_);
        server_fd_ = -1;
    }
}

bool TCPServer::acceptClient() {
    struct sockaddr_in client_addr;
    socklen_t addr_len = sizeof(client_addr);

    int fd = accept(server_fd_, (struct sockaddr*)&client_addr, &addr_len);
    if (fd < 0) {
        return false;
    }

    // Close existing client if any
    closeClient();

    // Set non-blocking
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);

    client_fd_ = fd;
    receive_buffer_.clear();

    std::cout << "GUI client connected" << std::endl;
    return true;
}

void TCPServer::closeClient() {
    if (client_fd_ >= 0) {
        close(client_fd_);
        client_fd_ = -1;
        receive_buffer_.clear();
        std::cout << "GUI client disconnected" << std::endl;
    }
}

bool TCPServer::poll(int timeout_ms) {
    std::vector<struct pollfd> fds;

    // Always poll server socket for new connections
    if (server_fd_ >= 0) {
        struct pollfd pfd;
        pfd.fd = server_fd_;
        pfd.events = POLLIN;
        fds.push_back(pfd);
    }

    // Poll client socket for data
    if (client_fd_ >= 0) {
        struct pollfd pfd;
        pfd.fd = client_fd_;
        pfd.events = POLLIN;
        fds.push_back(pfd);
    }

    if (fds.empty()) {
        return false;
    }

    int ret = ::poll(fds.data(), fds.size(), timeout_ms);
    if (ret <= 0) {
        return false;
    }

    bool hasCommand = false;

    for (const auto& pfd : fds) {
        if (pfd.revents & POLLIN) {
            if (pfd.fd == server_fd_) {
                acceptClient();
            } else if (pfd.fd == client_fd_) {
                if (readFromClient()) {
                    hasCommand = true;
                }
            }
        }
        if (pfd.revents & (POLLERR | POLLHUP)) {
            if (pfd.fd == client_fd_) {
                closeClient();
            }
        }
    }

    return hasCommand;
}

bool TCPServer::readFromClient() {
    char buffer[1024];
    ssize_t n = read(client_fd_, buffer, sizeof(buffer) - 1);

    if (n <= 0) {
        if (n == 0 || (errno != EAGAIN && errno != EWOULDBLOCK)) {
            closeClient();
        }
        return false;
    }

    buffer[n] = '\0';
    receive_buffer_ += buffer;

    // Look for complete JSON message (newline-delimited)
    size_t pos;
    bool hasCommand = false;
    while ((pos = receive_buffer_.find('\n')) != std::string::npos) {
        std::string msg = receive_buffer_.substr(0, pos);
        receive_buffer_.erase(0, pos + 1);

        TCPCommand cmd;
        if (parseJSON(msg, cmd)) {
            last_command_ = cmd;
            hasCommand = true;
        }
    }

    return hasCommand;
}

bool TCPServer::parseJSON(const std::string& json, TCPCommand& cmd) {
    // Simple JSON parsing for our protocol
    cmd.valid = false;

    // Find "cmd" field
    std::regex cmd_regex("\"cmd\"\\s*:\\s*\"([^\"]+)\"");
    std::smatch match;
    if (std::regex_search(json, match, cmd_regex)) {
        cmd.cmd = match[1];
        cmd.valid = true;
    }

    // Find "pos" field
    std::regex pos_regex("\"pos\"\\s*:\\s*([\\d.\\-]+)");
    if (std::regex_search(json, match, pos_regex)) {
        cmd.position = std::stof(match[1]);
    }

    // Find "vel" field
    std::regex vel_regex("\"vel\"\\s*:\\s*([\\d.\\-]+)");
    if (std::regex_search(json, match, vel_regex)) {
        cmd.velocity = std::stof(match[1]);
    }

    return cmd.valid;
}

TCPCommand TCPServer::getCommand() {
    TCPCommand cmd = last_command_;
    last_command_ = TCPCommand();
    return cmd;
}

bool TCPServer::sendStatus(float position_rad, float velocity_rad_s,
                           const std::string& mode, bool connected) {
    if (client_fd_ < 0) {
        return false;
    }

    std::ostringstream ss;
    ss << "{\"pos\":" << position_rad
       << ",\"vel\":" << velocity_rad_s
       << ",\"mode\":\"" << mode << "\""
       << ",\"connected\":" << (connected ? "true" : "false")
       << "}\n";

    std::string msg = ss.str();
    ssize_t n = write(client_fd_, msg.c_str(), msg.size());

    if (n < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
            closeClient();
        }
        return false;
    }

    return true;
}
