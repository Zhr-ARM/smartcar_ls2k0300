#include "driver/vision/vision_assistant_udp.h"

#include "seekfree_assistant_interface.h"

#include <arpa/inet.h>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <sys/socket.h>
#include <unistd.h>

namespace
{
static std::mutex g_lock;
static int g_sock = -1;
static bool g_ready = false;
static sockaddr_in g_peer_addr{};

static uint32 vision_assistant_udp_send(const uint8 *buff, uint32 length)
{
    if (buff == nullptr || length == 0)
    {
        return 0;
    }
    std::lock_guard<std::mutex> lk(g_lock);
    if (!g_ready || g_sock < 0)
    {
        return length;
    }
    const ssize_t sent = sendto(g_sock,
                                buff,
                                length,
                                0,
                                reinterpret_cast<const sockaddr *>(&g_peer_addr),
                                sizeof(g_peer_addr));
    if (sent < 0)
    {
        return length;
    }
    if (static_cast<uint32>(sent) >= length)
    {
        return 0;
    }
    return length - static_cast<uint32>(sent);
}

static uint32 vision_assistant_udp_recv(uint8 *buff, uint32 length)
{
    (void)buff;
    (void)length;
    return 0;
}
} // namespace

bool vision_assistant_udp_init(const char *ip, uint16 port)
{
    if (ip == nullptr || ip[0] == '\0' || port == 0)
    {
        return false;
    }

    std::lock_guard<std::mutex> lk(g_lock);
    if (g_sock >= 0)
    {
        close(g_sock);
        g_sock = -1;
        g_ready = false;
    }

    g_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (g_sock < 0)
    {
        printf("[ASSISTANT_UDP] socket failed err=%d\r\n", errno);
        return false;
    }

    std::memset(&g_peer_addr, 0, sizeof(g_peer_addr));
    g_peer_addr.sin_family = AF_INET;
    g_peer_addr.sin_port = htons(port);
    if (inet_pton(AF_INET, ip, &g_peer_addr.sin_addr) != 1)
    {
        printf("[ASSISTANT_UDP] invalid ip: %s\r\n", ip);
        close(g_sock);
        g_sock = -1;
        return false;
    }

    seekfree_assistant_interface_init(vision_assistant_udp_send, vision_assistant_udp_recv);
    g_ready = true;
    return true;
}

void vision_assistant_udp_cleanup()
{
    std::lock_guard<std::mutex> lk(g_lock);
    if (g_sock >= 0)
    {
        close(g_sock);
        g_sock = -1;
    }
    g_ready = false;
}

bool vision_assistant_udp_is_ready()
{
    std::lock_guard<std::mutex> lk(g_lock);
    return g_ready;
}
