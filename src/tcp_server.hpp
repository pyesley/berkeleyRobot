#pragma once

#include <string>
#include <functional>
#include <vector>
#include <cstdint>

struct TCPCommand {
    std::string cmd;
    float position = 0.0f;
    float velocity = 0.0f;
    bool valid = false;
};

class TCPServer {
public:
    TCPServer();
    ~TCPServer();

    // Start server on specified port
    bool start(int port);

    // Stop server
    void stop();

    // Poll for new connections and data (non-blocking)
    // Returns true if there's a new command
    bool poll(int timeout_ms);

    // Get the last received command
    TCPCommand getCommand();

    // Send status to connected client
    bool sendStatus(float position_rad, float velocity_rad_s,
                    const std::string& mode, bool connected);

    // Check if a client is connected
    bool hasClient() const { return client_fd_ >= 0; }

    // Get server socket fd for external polling
    int getFd() const { return server_fd_; }

private:
    int server_fd_;
    int client_fd_;
    int port_;
    std::string receive_buffer_;
    TCPCommand last_command_;

    bool acceptClient();
    void closeClient();
    bool readFromClient();
    bool parseJSON(const std::string& json, TCPCommand& cmd);
};
