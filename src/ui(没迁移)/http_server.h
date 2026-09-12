#pragma once

#include <functional>
#include <map>
#include <string>

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <winsock2.h>

namespace mss {

struct HttpRequest {
    std::string method;
    std::string path;
    std::map<std::string, std::string> query;
    std::string body;
};

struct HttpResponse {
    int status = 200;
    std::string contentType;
    std::string body;
};

class HttpServer {
public:
    using Handler = std::function<HttpResponse(const HttpRequest&)>;

    ~HttpServer();
    void setHandler(Handler h);
    bool start(int port);
    void stop();
    void run();
    void requestStop();

private:
    Handler handler_;
    SOCKET listenSock_ = INVALID_SOCKET;
    bool running_ = false;

    void handleClient(SOCKET client);
    static std::string urlDecode(const std::string& s);
    static int hexVal(char c);
    static const char* statusText(int code);
};

}  // namespace mss