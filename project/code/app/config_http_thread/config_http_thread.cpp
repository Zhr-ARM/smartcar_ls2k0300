#include "config_http_thread.h"

#include "driver/config/smartcar_config.h"

#include <algorithm>
#include <atomic>
#include <cctype>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <netinet/in.h>
#include <sstream>
#include <string>
#include <sys/select.h>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <vector>

namespace
{
constexpr int kListenPort = 18080;
constexpr int kListenBacklog = 4;
constexpr int kAcceptPollMs = 200;
constexpr size_t kMaxRequestBytes = 1024 * 1024;

std::thread g_config_http_thread;
std::atomic<bool> g_running(false);
int g_listen_fd = -1;

std::string trim(const std::string &text)
{
    size_t begin = 0;
    while (begin < text.size() && std::isspace(static_cast<unsigned char>(text[begin])))
    {
        ++begin;
    }
    size_t end = text.size();
    while (end > begin && std::isspace(static_cast<unsigned char>(text[end - 1])))
    {
        --end;
    }
    return text.substr(begin, end - begin);
}

void close_listen_fd()
{
    if (g_listen_fd >= 0)
    {
        close(g_listen_fd);
        g_listen_fd = -1;
    }
}

std::string json_escape(const std::string &text)
{
    std::string out;
    out.reserve(text.size() + 16);
    for (const char ch : text)
    {
        switch (ch)
        {
            case '\\': out += "\\\\"; break;
            case '"': out += "\\\""; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default: out.push_back(ch); break;
        }
    }
    return out;
}

void write_response(int client_fd,
                    int status_code,
                    const char *status_text,
                    const char *content_type,
                    const std::string &body)
{
    std::ostringstream response;
    response << "HTTP/1.1 " << status_code << ' ' << status_text << "\r\n"
             << "Content-Type: " << content_type << "\r\n"
             << "Content-Length: " << body.size() << "\r\n"
             << "Connection: close\r\n"
             << "Cache-Control: no-store\r\n"
             << "Access-Control-Allow-Origin: *\r\n"
             << "Access-Control-Allow-Methods: GET, POST, OPTIONS\r\n"
             << "Access-Control-Allow-Headers: Content-Type\r\n\r\n"
             << body;
    const std::string payload = response.str();
    size_t sent_total = 0;
    while (sent_total < payload.size())
    {
        const ssize_t sent = send(client_fd,
                                  payload.data() + sent_total,
                                  payload.size() - sent_total,
                                  0);
        if (sent <= 0)
        {
            break;
        }
        sent_total += static_cast<size_t>(sent);
    }
}

bool read_http_request(int client_fd,
                       std::string *request_header,
                       std::string *request_body)
{
    std::string buffer;
    buffer.reserve(4096);
    char temp[4096];
    size_t header_end = std::string::npos;
    size_t content_length = 0;

    while (buffer.size() < kMaxRequestBytes)
    {
        const ssize_t received = recv(client_fd, temp, sizeof(temp), 0);
        if (received <= 0)
        {
            return false;
        }
        buffer.append(temp, static_cast<size_t>(received));
        header_end = buffer.find("\r\n\r\n");
        if (header_end == std::string::npos)
        {
            continue;
        }

        const std::string header_text = buffer.substr(0, header_end + 4);
        const size_t content_length_pos = header_text.find("Content-Length:");
        if (content_length_pos != std::string::npos)
        {
            const size_t line_end = header_text.find("\r\n", content_length_pos);
            const std::string value = trim(header_text.substr(content_length_pos + 15,
                                                              line_end - (content_length_pos + 15)));
            content_length = static_cast<size_t>(std::max(0, std::atoi(value.c_str())));
        }
        const size_t body_begin = header_end + 4;
        if (buffer.size() >= body_begin + content_length)
        {
            *request_header = buffer.substr(0, body_begin);
            *request_body = buffer.substr(body_begin, content_length);
            return true;
        }
    }
    return false;
}

void handle_client(int client_fd)
{
    std::string header;
    std::string body;
    if (!read_http_request(client_fd, &header, &body))
    {
        write_response(client_fd, 400, "Bad Request", "text/plain; charset=utf-8", "bad request");
        return;
    }

    const size_t first_line_end = header.find("\r\n");
    const std::string first_line = (first_line_end == std::string::npos) ? header : header.substr(0, first_line_end);
    std::istringstream first_line_stream(first_line);
    std::string method;
    std::string path;
    std::string version;
    first_line_stream >> method >> path >> version;

    if (method == "OPTIONS")
    {
        write_response(client_fd, 204, "No Content", "text/plain; charset=utf-8", "");
        return;
    }

    if (method == "GET" && path == "/api/config/current")
    {
        std::string toml_text;
        std::string loaded_path;
        std::string error_message;
        if (!smartcar_config_read_loaded_text(&toml_text, &loaded_path, &error_message))
        {
            write_response(client_fd,
                           500,
                           "Internal Server Error",
                           "application/json; charset=utf-8",
                           std::string("{\"ok\":false,\"message\":\"") + json_escape(error_message) + "\"}");
            return;
        }

        std::ostringstream json;
        json << "{\"ok\":true,"
             << "\"loaded_path\":\"" << json_escape(loaded_path) << "\","
             << "\"toml_text\":\"" << json_escape(toml_text) << "\"}";
        write_response(client_fd, 200, "OK", "application/json; charset=utf-8", json.str());
        return;
    }

    if (method == "POST" && path == "/api/config/apply")
    {
        std::vector<std::string> restart_required_keys;
        std::string error_message;
        if (!smartcar_config_apply_toml_text(body, &restart_required_keys, &error_message))
        {
            write_response(client_fd,
                           400,
                           "Bad Request",
                           "application/json; charset=utf-8",
                           std::string("{\"ok\":false,\"message\":\"") + json_escape(error_message) + "\"}");
            return;
        }

        std::ostringstream restart_json;
        restart_json << "[";
        for (size_t i = 0; i < restart_required_keys.size(); ++i)
        {
            if (i > 0)
            {
                restart_json << ",";
            }
            restart_json << "\"" << json_escape(restart_required_keys[i]) << "\"";
        }
        restart_json << "]";

        std::ostringstream json;
        json << "{\"ok\":true,"
             << "\"message\":\"applied\","
             << "\"loaded_path\":\"" << json_escape(smartcar_config_loaded_path()) << "\","
             << "\"restart_required\":" << (restart_required_keys.empty() ? "false" : "true") << ","
             << "\"restart_required_keys\":" << restart_json.str()
             << "}";
        write_response(client_fd, 200, "OK", "application/json; charset=utf-8", json.str());
        return;
    }

    if (method == "GET" && path == "/healthz")
    {
        write_response(client_fd, 200, "OK", "text/plain; charset=utf-8", "ok");
        return;
    }

    write_response(client_fd, 404, "Not Found", "text/plain; charset=utf-8", "not found");
}

void server_loop()
{
    while (g_running.load())
    {
        fd_set read_fds;
        FD_ZERO(&read_fds);
        FD_SET(g_listen_fd, &read_fds);
        struct timeval timeout{};
        timeout.tv_sec = 0;
        timeout.tv_usec = kAcceptPollMs * 1000;
        const int ready = select(g_listen_fd + 1, &read_fds, nullptr, nullptr, &timeout);
        if (ready <= 0)
        {
            continue;
        }

        struct sockaddr_in client_addr{};
        socklen_t client_len = sizeof(client_addr);
        const int client_fd = accept(g_listen_fd, reinterpret_cast<struct sockaddr *>(&client_addr), &client_len);
        if (client_fd < 0)
        {
            continue;
        }
        handle_client(client_fd);
        close(client_fd);
    }
}
} // namespace

bool config_http_thread_init()
{
    if (g_running.load())
    {
        return true;
    }

    g_listen_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (g_listen_fd < 0)
    {
        return false;
    }

    int reuse = 1;
    setsockopt(g_listen_fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    struct sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(kListenPort);
    if (bind(g_listen_fd, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) != 0)
    {
        close_listen_fd();
        return false;
    }
    if (listen(g_listen_fd, kListenBacklog) != 0)
    {
        close_listen_fd();
        return false;
    }

    g_running.store(true);
    g_config_http_thread = std::thread(server_loop);
    return true;
}

void config_http_thread_cleanup()
{
    g_running.store(false);
    close_listen_fd();
    if (g_config_http_thread.joinable())
    {
        g_config_http_thread.join();
    }
}

bool config_http_thread_is_running()
{
    return g_running.load();
}
