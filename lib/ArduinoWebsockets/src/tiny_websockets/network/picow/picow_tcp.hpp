#pragma once

#ifdef ARDUINO_RASPBERRY_PI_PICO_W

#include <tiny_websockets/internals/ws_common.hpp>
#include <tiny_websockets/network/tcp_client.hpp>
#include <tiny_websockets/network/tcp_server.hpp>

#include <WiFi.h>

namespace websockets { namespace network {

    class PicoWTcpClient : public TcpClient {
        public:
            PicoWTcpClient(WiFiClient c) : client(c) {}
            PicoWTcpClient() {}

            bool connect(const WSString& host, const int port) {
                yield();
                auto didConnect = client.connect(host.c_str(), port);
                client.setNoDelay(true);
                return didConnect;
            }

            bool poll() {
                yield();
                return client.available();
            }

            bool available() override {
                return client.connected();
            }

            void send(const WSString& data) override {
                yield();
                client.write(reinterpret_cast<uint8_t*>(const_cast<char*>(data.c_str())), data.size());
                yield();
            }

            void send(const WSString&& data) override {
                yield();
                client.write(reinterpret_cast<uint8_t*>(const_cast<char*>(data.c_str())), data.size());
                yield();
            }

            void send(const uint8_t* data, const uint32_t len) override {
                yield();
                client.write(data, len);
                yield();
            }

            WSString readLine() override {
                WSString line = "";

                int ch = -1;

                const uint64_t millisBeforeReadingHeaders = millis();
                while (ch != '\n' && available()) {
                    if (millis() - millisBeforeReadingHeaders > _CONNECTION_TIMEOUT) return "";
                    ch = client.read();
                    if (ch < 0) continue;
                    line += (char) ch;
                }

                return line;
            }

            uint32_t read(uint8_t* buffer, const uint32_t len) override {
                yield();
                return client.read(buffer, len);
            }

            void close() override {
                yield();
                client.stop();
            }

            ~PicoWTcpClient() {
                client.stop();
            }

        protected:
            WiFiClient client;

            int getSocket() const override {
                return -1;
            }
    };

    #define DUMMY_PORT 0
    #define CLOSED 0

    class PicoWTcpServer : public TcpServer {
        public:
            PicoWTcpServer() : server(DUMMY_PORT) {}
            bool poll() override {
                yield();
                return server.hasClient();
            }

            bool listen(const uint16_t port) override {
                yield();
                server.begin(port);
                return available();
            }

            TcpClient* accept() override {
                while (available()) {
                    auto client = server.available();
                    if (client) {
                        return new PicoWTcpClient(client);
                    }
                }
                return new PicoWTcpClient;
            }

            bool available() override {
                yield();
                return server.status() != CLOSED;
            }

            void close() override {
                yield();
                server.close();
            }

            ~PicoWTcpServer() {
                if (available()) close();
            }

        protected:
            int getSocket() const override {
                return -1;
            }

        private:
            WiFiServer server;
    };


}} // websockets::network

#endif // #ifdef ARDUINO_RASPBERRY_PI_PICO_W
