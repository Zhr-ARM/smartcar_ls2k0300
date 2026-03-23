#include "zf_driver_tcp_client.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <cstring>
#include <arpa/inet.h>
#include <poll.h>
#include <chrono>
#include <cstdint>
#include <netinet/tcp.h>

int set_nonblocking(int fd) 
{
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags == -1) return -1;
    return fcntl(fd, F_SETFL, flags | O_NONBLOCK);
}


static sockaddr_in server_addr;
static int tcp_client_socket = -1;
static char server_ip[64] = {0};
static uint32 server_port = 0;

static constexpr int64_t TCP_RECONNECT_INTERVAL_MS = 1000;
static constexpr int32 TCP_CONNECT_TIMEOUT_MS = 300;
static constexpr int32 TCP_SEND_POLL_TIMEOUT_MS = 2;
static constexpr int32 TCP_SEND_MAX_WAIT_ROUNDS = 8;
static constexpr int64_t TCP_ERROR_LOG_INTERVAL_MS = 500;

static int64_t g_last_reconnect_ms = 0;
static int64_t g_last_error_log_ms = 0;

#ifndef MSG_NOSIGNAL
#define MSG_NOSIGNAL 0
#endif

static int64_t now_ms()
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

static void tcp_client_close_socket()
{
    if (tcp_client_socket >= 0)
    {
        close(tcp_client_socket);
        tcp_client_socket = -1;
    }
}

static void tcp_client_log_error_rate_limited(const char *tag, int err, uint32 length, uint32 sent)
{
    int64_t now = now_ms();
    if (now - g_last_error_log_ms < TCP_ERROR_LOG_INTERVAL_MS)
    {
        return;
    }
    g_last_error_log_ms = now;
    printf("%s err=%d(%s) len=%u sent=%u\r\n", tag, err, strerror(err), length, sent);
}

static int tcp_client_connect_socket()
{
    tcp_client_close_socket();

    tcp_client_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (tcp_client_socket < 0)
    {
        printf("Failed to create socket\r\n");
        return -1;
    }

    int one = 1;
    setsockopt(tcp_client_socket, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    setsockopt(tcp_client_socket, SOL_SOCKET, SO_KEEPALIVE, &one, sizeof(one));

    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = inet_addr(server_ip);
    server_addr.sin_port = htons(server_port);

    if (set_nonblocking(tcp_client_socket) < 0)
    {
        perror("set_nonblocking");
        tcp_client_close_socket();
        return -1;
    }

    int ret = connect(tcp_client_socket, (struct sockaddr *)&server_addr, sizeof(server_addr));
    if (ret < 0 && errno != EINPROGRESS)
    {
        perror("connect() error");
        tcp_client_close_socket();
        return -1;
    }

    if (ret < 0)
    {
        struct pollfd pfd;
        pfd.fd = tcp_client_socket;
        pfd.events = POLLOUT;
        pfd.revents = 0;

        int poll_ret = poll(&pfd, 1, TCP_CONNECT_TIMEOUT_MS);
        if (poll_ret <= 0)
        {
            if (poll_ret == 0)
            {
                errno = ETIMEDOUT;
            }
            perror("connect() timeout/error");
            tcp_client_close_socket();
            return -1;
        }

        int sock_err = 0;
        socklen_t len = sizeof(sock_err);
        if (getsockopt(tcp_client_socket, SOL_SOCKET, SO_ERROR, &sock_err, &len) < 0 || sock_err != 0)
        {
            if (sock_err != 0)
            {
                errno = sock_err;
            }
            perror("connect() async error");
            tcp_client_close_socket();
            return -1;
        }
    }

    return 0;
}

static bool tcp_client_try_reconnect_if_needed()
{
    if (tcp_client_socket >= 0)
    {
        return true;
    }

    if (server_ip[0] == '\0' || server_port == 0)
    {
        return false;
    }

    int64_t now = now_ms();
    if (now - g_last_reconnect_ms < TCP_RECONNECT_INTERVAL_MS)
    {
        return false;
    }

    g_last_reconnect_ms = now;
    if (tcp_client_connect_socket() == 0)
    {
        printf("tcp reconnect ok\r\n");
        return true;
    }

    return false;
}

int8 tcp_client_init(const char *ip_addr, uint32 port)
{
    if (ip_addr == nullptr || port == 0)
    {
        return -1;
    }

    ::snprintf(server_ip, sizeof(server_ip), "%s", ip_addr);
    server_port = port;

    printf("Wait connect tcp server\r\n");

    if (tcp_client_connect_socket() != 0)
    {
        return -1;
    }

    return 0;
}


uint32 tcp_client_send_data(const uint8 *buff, uint32 length)
{
    if (buff == nullptr || length == 0)
    {
        return 0;
    }

    if (!tcp_client_try_reconnect_if_needed())
    {
        return length;
    }

    uint32 sent_total = 0;
    int wait_rounds = 0;

    while (sent_total < length)
    {
        ssize_t str_len = send(tcp_client_socket, buff + sent_total, length - sent_total, MSG_NOSIGNAL);
        if (str_len > 0)
        {
            sent_total += static_cast<uint32>(str_len);
            wait_rounds = 0;
            continue;
        }

        if (str_len == 0)
        {
            tcp_client_log_error_rate_limited("send() closed", ECONNRESET, length, sent_total);
            tcp_client_close_socket();
            return length;
        }

        if (errno == EINTR)
        {
            continue;
        }

        if (errno == EAGAIN || errno == EWOULDBLOCK)
        {
            struct pollfd pfd;
            pfd.fd = tcp_client_socket;
            pfd.events = POLLOUT;
            pfd.revents = 0;

            int poll_ret = poll(&pfd, 1, TCP_SEND_POLL_TIMEOUT_MS);
            if (poll_ret > 0 && (pfd.revents & POLLOUT))
            {
                continue;
            }

            wait_rounds++;
            if (wait_rounds < TCP_SEND_MAX_WAIT_ROUNDS)
            {
                continue;
            }

            tcp_client_log_error_rate_limited("send() timeout", errno, length, sent_total);
            tcp_client_close_socket();
            return length;
        }

        tcp_client_log_error_rate_limited("send() fatal", errno, length, sent_total);
        tcp_client_close_socket();
        return length;
    }

    return 0;
}



uint32 tcp_client_read_data(uint8 *buff, uint32 length)
{
    if (buff == nullptr || length == 0)
    {
        return 0;
    }

    if (!tcp_client_try_reconnect_if_needed())
    {
        return 0;
    }

    // 接收服务器响应
    ssize_t str_len = recv(tcp_client_socket, buff, length, 0);

    if (str_len == -1)
    {
        if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)
        {   // 暂时无数据可读，稍后重试或处理其他逻辑。
            return 0;
        }

        tcp_client_log_error_rate_limited("recv() fatal", errno, length, 0);
        tcp_client_close_socket();
        return 0;
    }

    if (str_len == 0)
    {
        tcp_client_close_socket();
        return 0;
    }

    return static_cast<uint32>(str_len);
}



