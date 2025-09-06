#include <iostream>
#include <string>
#include <vector>
#include <sstream>
#include <curl/curl.h>
// #include <json/json.h>  // Make sure you have the jsoncpp library installed
#include "/usr/include/jsoncpp/json/json.h"

//g++ -o bus_servo_http bus_servo_http.cpp -lcurl -ljsoncpp
class BusServoHttp {
public:
    BusServoHttp(const std::string &address) : address(address) {
        curl_global_init(CURL_GLOBAL_DEFAULT);
        session = curl_easy_init();
    }

    ~BusServoHttp() {
        if (session) {
            curl_easy_cleanup(session);
        }
        curl_global_cleanup();
    }

    std::string run_command(const std::string &command, const std::vector<std::string> &params = {}) {
        std::string url = address + "/command?method=" + command;
        if (!params.empty()) {
            url += "&params=" + join(params, ',');
        }

        curl_easy_setopt(session, CURLOPT_URL, url.c_str());
        curl_easy_setopt(session, CURLOPT_WRITEFUNCTION, WriteCallback);
        curl_easy_setopt(session, CURLOPT_WRITEDATA, &readBuffer);
        CURLcode res = curl_easy_perform(session);

        if (res != CURLE_OK) {
            std::cerr << "Request failed: " << curl_easy_strerror(res) << std::endl;
            curl_easy_cleanup(session);
            session = curl_easy_init();
            return "";
        }

        long http_code = 0;
        curl_easy_getinfo(session, CURLINFO_RESPONSE_CODE, &http_code);
        if (http_code != 200) {
            std::cerr << "HTTP error code: " << http_code << std::endl;
            return "";
        }

        char *ct;
        res = curl_easy_getinfo(session, CURLINFO_CONTENT_TYPE, &ct);
        if ((res == CURLE_OK) && ct) {
            std::string contentType(ct);
            if (contentType == "application/json") {
                Json::Value jsonData;
                Json::Reader jsonReader;
                if (jsonReader.parse(readBuffer, jsonData)) {
                    return jsonData.toStyledString();
                } else {
                    std::cerr << "Failed to parse JSON response" << std::endl;
                    return "";
                }
            } else {
                return readBuffer;
            }
        }

        return readBuffer;
    }

    std::string run(int id, int p, int servo_run_time = 1000) {
        return run_command("run", {std::to_string(id), std::to_string(p), std::to_string(servo_run_time)});
    }

    std::string run_mult(const std::string &pp, int servo_run_time) {
        return run_command("run_mult", {pp, std::to_string(servo_run_time)});
    }

    std::string run_add_or_dec(int id, int speed) {
        return run_command("run_add_or_dec", {std::to_string(id), std::to_string(speed)});
    }

    std::string stop(int id) {
        return run_command("stop", {std::to_string(id)});
    }

    std::string set_ID(int old_id, int new_id) {
        return run_command("set_ID", {std::to_string(old_id), std::to_string(new_id)});
    }

    std::string get_ID(int id) {
        return run_command("get_ID", {std::to_string(id)});
    }

    std::string set_mode(int id, int mode, int speed = 0) {
        return run_command("set_mode", {std::to_string(id), std::to_string(mode), std::to_string(speed)});
    }

    std::string load(int id) {
        return run_command("load", {std::to_string(id)});
    }

    std::string unload(int id) {
        return run_command("unload", {std::to_string(id)});
    }

    std::string servo_receive_handle() {
        return run_command("servo_receive_handle");
    }

    int get_position(int id) {
        return std::stoi(run_command("get_position", {std::to_string(id)}));
    }

    Json::Value get_positions() {
        std::string pos_string = run_command("get_positions");
        Json::Value jsonData;
        Json::Reader jsonReader;
        if (jsonReader.parse(pos_string, jsonData)) {
            return jsonData;
        } else {
            throw std::runtime_error("Could not get position from string " + pos_string);
        }
    }

    void set_positions(const std::vector<int> &goal_positions, int servo_run_time) {
        if (goal_positions.size() != 6) {
            throw std::runtime_error("goal_positions must have exactly 6 elements");
        }
        std::vector<std::string> params;
        for (int pos : goal_positions) {
            params.push_back(std::to_string(pos));
        }
        params.push_back(std::to_string(servo_run_time));
        run_command("set_positions", params);
    }

    std::string get_vin(int id) {
        return run_command("get_vin", {std::to_string(id)});
    }

    std::string adjust_offset(int id, int offset) {
        return run_command("adjust_offset", {std::to_string(id), std::to_string(offset)});
    }

    std::string save_offset(int id) {
        return run_command("save_offset", {std::to_string(id)});
    }

    std::string get_offset(int id) {
        return run_command("get_offset", {std::to_string(id)});
    }

private:
    std::string address;
    CURL *session;
    std::string readBuffer;

    static size_t WriteCallback(void *contents, size_t size, size_t nmemb, void *userp) {
        ((std::string *)userp)->append((char *)contents, size * nmemb);
        return size * nmemb;
    }

    std::string join(const std::vector<std::string> &elements, char delimiter) {
        std::ostringstream os;
        for (const auto &elem : elements) {
            if (&elem != &elements[0]) {
                os << delimiter;
            }
            os << elem;
        }
        return os.str();
    }
};

int main() {
    BusServoHttp servo("http://192.168.1.121");
    std::string response = servo.run(6, 200);
    std::cout << "Response: " << response << std::endl;
    return 0;
}
