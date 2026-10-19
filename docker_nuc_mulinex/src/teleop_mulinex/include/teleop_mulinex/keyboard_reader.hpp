#pragma once

#include <cstring>
#include <stdexcept>
#include <termios.h>
#include <unistd.h>

class KeyboardReader {
public:
    KeyboardReader(): kfd(0)
    {
        tcgetattr(kfd, &cooked);
        memcpy(&raw, &cooked, sizeof(struct termios));
        raw.c_lflag &= ~(ICANON | ECHO);
        raw.c_cc[VEOL] = 1;
        raw.c_cc[VEOF] = 2;
        raw.c_cc[VMIN] = 0;
        raw.c_cc[VTIME] = 0;
        tcsetattr(kfd, TCSANOW, &raw);
    }

    bool read_one(char* c)
    {
        int rc = read(kfd, c, 1);
        if (rc < 0) {
            throw std::runtime_error("read failed");
        }
        return rc == 1;
    }

    void shutdown() { tcsetattr(kfd, TCSANOW, &cooked); }

private:
    int kfd;
    struct termios cooked, raw;
};
