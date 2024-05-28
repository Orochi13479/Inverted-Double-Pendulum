#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>
#include <websocketpp/client.hpp>

#include <iostream>
#include <thread>
#include <chrono>

typedef websocketpp::server<websocketpp::config::asio> server;
typedef websocketpp::client<websocketpp::config::asio> client;

// Define a custom handler for the server
class websocket_server
{
public:
    websocket_server()
    {
        // Initialize the server
        m_server.init_asio();

        // Set the open handler
        m_server.set_open_handler(bind(&websocket_server::on_open, this, std::placeholders::_1));
        // Set the message handler
        m_server.set_message_handler(bind(&websocket_server::on_message, this, std::placeholders::_1, std::placeholders::_2));
    }

    void on_open(websocketpp::connection_hdl hdl)
    {
        std::cout << "Server: Connection opened." << std::endl;
        m_hdl = hdl;
    }

    void on_message(websocketpp::connection_hdl hdl, server::message_ptr msg)
    {
        std::cout << "Server: Received message: " << msg->get_payload() << std::endl;
        m_server.send(hdl, msg->get_payload(), msg->get_opcode());
    }

    void run(uint16_t port)
    {
        // Listen on the specified port
        m_server.listen(port);
        // Start accepting connections
        m_server.start_accept();
        // Start the ASIO io_service run loop
        m_server.run();
    }

private:
    server m_server;
    websocketpp::connection_hdl m_hdl;
};

void run_server()
{
    websocket_server server_instance;
    server_instance.run(9002);
}

// Define a custom handler for the client
class websocket_client
{
public:
    websocket_client()
    {
        // Initialize the client
        m_client.init_asio();

        // Set the open handler
        m_client.set_open_handler(bind(&websocket_client::on_open, this, std::placeholders::_1));
        // Set the message handler
        m_client.set_message_handler(bind(&websocket_client::on_message, this, std::placeholders::_1, std::placeholders::_2));
    }

    void on_open(websocketpp::connection_hdl hdl)
    {
        std::cout << "Client: Connection opened." << std::endl;
        m_hdl = hdl;
        m_client.send(hdl, "Hello, WebSocket++!", websocketpp::frame::opcode::text);
    }

    void on_message(websocketpp::connection_hdl hdl, client::message_ptr msg)
    {
        std::cout << "Client: Received message: " << msg->get_payload() << std::endl;
    }

    void run(const std::string &uri)
    {
        websocketpp::lib::error_code ec;
        client::connection_ptr con = m_client.get_connection(uri, ec);

        if (ec)
        {
            std::cout << "Client: Could not create connection: " << ec.message() << std::endl;
            return;
        }

        m_client.connect(con);
        m_client.run();
    }

private:
    client m_client;
    websocketpp::connection_hdl m_hdl;
};

void run_client()
{
    websocket_client client_instance;
    client_instance.run("ws://localhost:9002");
}

int main()
{
    // Start the server in a separate thread
    std::thread server_thread(run_server);

    // Give the server some time to start
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // Start the client
    run_client();

    // Wait for the server thread to finish
    server_thread.join();

    return 0;
}
