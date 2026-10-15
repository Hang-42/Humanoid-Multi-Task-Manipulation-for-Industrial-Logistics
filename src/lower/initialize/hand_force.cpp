// Exclusive Revo1 Advanced CAN configuration through installed vendor SDKs.
// Default operation is read-only. Explicit writes are limited to verified
// 500/600 mA protection parameters for the five grip joints; AUX is retained.
// No body motor, finger movement/current command, or calibration is transmitted.
#include "canbus_sdk/canbus_sdk.h"
#include "canbus_sdk/config_parser.h"
#include "stark-sdk.h"

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <deque>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <sys/file.h>
#include <fcntl.h>
#include <unistd.h>

using namespace std::chrono_literals;
namespace cb = canbus_sdk;

struct Packet { uint32_t id; size_t size; std::array<uint8_t, 8> data; };
struct Session {
    std::mutex mutex;
    std::condition_variable changed;
    std::deque<Packet> packets;
    std::string bus;
    uint8_t slave = 0;
    std::atomic<bool> capture_write{false};
    std::atomic<const char*> capture_operation{"none"};
    std::atomic<int> permitted_parameter{-1};
    std::atomic<unsigned> permitted_value{0}, writes{0};
    std::atomic<unsigned> tx{0}, rx{0};
};
static Session* active = nullptr;

static void require(bool ok, const std::string& error) {
    if (!ok) throw std::runtime_error(error);
}

static void reject_running_controller() {
    for (const auto& entry : std::filesystem::directory_iterator("/proc")) {
        const auto name = entry.path().filename().string();
        if (name.empty() || name.find_first_not_of("0123456789") != std::string::npos) continue;
        std::ifstream input(entry.path() / "cmdline");
        std::string cmd((std::istreambuf_iterator<char>(input)), {});
        for (char& c : cmd) if (c == '\0') c = ' ';
        require(cmd.find("load_kuavo_real") == std::string::npos &&
                cmd.find("robot_bridge.py") == std::string::npos &&
                !(cmd.find("nodelet") != std::string::npos &&
                  cmd.find("manager") != std::string::npos),
                "A full-body controller or live bridge is running (pid " + name + ")");
    }
}

static void on_frame(cb::CanMessageFrame* frame, const cb::CallbackContext* context) {
    if (!frame) return;
    auto* session = static_cast<Session*>(context->userdata);
    const size_t size = cb::dlc_to_payload_length(frame->ctrl.rx.dlc);
    if (frame->id.EID == 0 && size <= 8) {
        Packet packet{};
        packet.id = frame->id.SID;
        packet.size = size;
        std::memcpy(packet.data.data(), frame->payload, size);
        std::lock_guard<std::mutex> lock(session->mutex);
        if (session->packets.size() >= 128) session->packets.pop_front();
        session->packets.push_back(packet);
        session->changed.notify_one();
    }
    cb::freeCanMessageFrame(frame);
}

static int32_t send_read(uint8_t slave, uint32_t id, const uint8_t* data, uintptr_t size) {
    if (active && active->capture_write) {
        std::printf("CAPTURED_WRITE_NOT_SENT operation=%s slave=%u id=0x%X size=%zu data=", active->capture_operation.load(), slave, id,
                    static_cast<size_t>(size));
        for (uintptr_t i = 0; i < size; ++i) std::printf("%02X", data[i]);
        std::printf("\n");
        return 0;
    }
    const auto command = (id >> 3) & 0x0f;
    bool configuration_write = false;
    if (active && command == 0x09 && size == 8) {
        configuration_write = id == ((uint32_t(slave) << 7) | 0x4e) &&
            active->permitted_parameter.load() == data[0] &&
            active->permitted_value.load() == (unsigned(data[1]) | (unsigned(data[2]) << 8));
        for (int i = 3; i < 8; ++i) configuration_write = configuration_write && data[i] == 0;
    }
    if (!active || slave != active->slave || id > 0x7ff || size > 8 ||
        (command != 0x05 && command != 0x06 && command != 0x07 && command != 0x08 &&
         command != 0x0a && command != 0x0b && !configuration_write)) {
        std::fprintf(stderr, "READ_ONLY_TX_REJECTED slave=%u id=0x%X size=%zu\n",
                     slave, id, static_cast<size_t>(size));
        return -1;
    }
    cb::CanMessageFrame frame{};
    frame.id.SID = id;
    frame.ctrl.tx.dlc = cb::payload_length_to_dlc(size);
    std::memcpy(frame.payload, data, size);
    std::printf("%s slave=%u id=0x%X size=%zu\n", configuration_write ? "PROTECTION_WRITE_TX" : "PARAMETER_READ_TX", slave, id,
                static_cast<size_t>(size));
    const auto result = cb::CanBusController::getInstance().sendMessage(active->bus, frame);
    if (!result.has_value()) return -1;
    ++active->tx;
    if (configuration_write) ++active->writes;
    return 0;
}

static unsigned read_protection(DeviceHandler* handler, Session& session, int finger) {
    const unsigned before_tx = session.tx, before_rx = session.rx;
    const unsigned value = stark_get_finger_protected_current(handler, session.slave,
                                static_cast<StarkFingerId>(finger));
    require(session.tx > before_tx && session.rx > before_rx && value > 0,
            "Protection-current read did not receive a device response");
    return value;
}

static void write_verified(DeviceHandler* handler, Session& session, int finger, unsigned value) {
    require(finger != 2 && finger >= 1 && finger <= 6 && (value == 500 || value == 600),
            "Only the five grip joints and 500/600 mA are allowed");
    session.permitted_value = value;
    session.permitted_parameter = 0x10 + finger - 1;
    const unsigned before = session.writes;
    stark_set_finger_protected_current(handler, session.slave,
                                      static_cast<StarkFingerId>(finger), value);
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (session.writes == before && std::chrono::steady_clock::now() < deadline)
        std::this_thread::sleep_for(10ms);
    session.permitted_parameter = -1;
    require(session.writes > before, "SDK did not transmit the allowed parameter write");
    for (int attempt = 0; attempt < 3; ++attempt) {
        std::this_thread::sleep_for(100ms);
        if (read_protection(handler, session, finger) == value) return;
    }
    throw std::runtime_error("Protection-current write readback mismatch");
}

static int32_t receive_read(uint8_t slave, uint32_t expected, uint8_t frames,
                            uint32_t* id, uint8_t* data, uintptr_t* size) {
    *size = 0;
    if (!active || slave != active->slave || !expected) return -1;
    const auto deadline = std::chrono::steady_clock::now() + 1500ms;
    const bool multi = ((expected >> 3) & 0x0f) == 0x0b;
    unsigned count = 0;
    std::unique_lock<std::mutex> lock(active->mutex);
    while (std::chrono::steady_clock::now() < deadline) {
        while (!active->packets.empty()) {
            const auto packet = active->packets.front();
            active->packets.pop_front();
            if (packet.id != expected) continue;
            if (*size + packet.size > 512) return -1;
            std::memcpy(data + *size, packet.data.data(), packet.size);
            *size += packet.size;
            ++count;
            const bool last = multi && packet.size >= 2 && (packet.data[1] & 0x80);
            if (last || (!multi && count >= (frames ? frames : 1))) {
                *id = expected;
                ++active->rx;
                return 0;
            }
        }
        active->changed.wait_until(lock, deadline);
    }
    std::fprintf(stderr, "PARAMETER_READ_TIMEOUT slave=%u id=0x%X\n", slave, expected);
    *size = 0;
    return -1;
}

int main(int argc, char** argv) {
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    const std::array<int, 5> grip_fingers{1, 3, 4, 5, 6};
    std::array<unsigned, 6> targets{};
    const bool set_mode = argc == 4 && std::strcmp(argv[1], "--set-grip-current") == 0 &&
                         (std::strcmp(argv[2], "500") == 0 || std::strcmp(argv[2], "600") == 0);
    bool restore_mode = argc == 4 && std::strcmp(argv[1], "--restore-grip-current") == 0;
    if (restore_mode) {
        const std::string values = argv[2];
        // Five exact 500/600 values, in thumb/index/middle/ring/pinky order.
        restore_mode = values.size() == 19;
        for (size_t i = 0; restore_mode && i < grip_fingers.size(); ++i) {
            const auto value = values.substr(i * 4, 3);
            restore_mode = (value == "500" || value == "600") &&
                           (i == 4 || values[i * 4 + 3] == ',');
            if (restore_mode) targets[grip_fingers[i] - 1] = value == "500" ? 500 : 600;
        }
    }
    const bool inspect_mode = argc == 3 && (std::strcmp(argv[1], "--read-only") == 0 ||
                                            std::strcmp(argv[1], "--capture-write") == 0);
    const char* side = (set_mode || restore_mode || inspect_mode) ? argv[argc - 1] : "";
    if ((!set_mode && !restore_mode && !inspect_mode) || (std::strcmp(side, "left") != 0 && std::strcmp(side, "right") != 0)) {
        std::fprintf(stderr, "Usage: %s {--read-only|--capture-write} {left|right}\n"
                             "       %s --set-grip-current {500|600} {left|right}\n"
                             "       %s --restore-grip-current V1,V3,V4,V5,V6 {left|right}\n", argv[0], argv[0], argv[0]);
        return 2;
    }
    const bool left = std::strcmp(side, "left") == 0;
    const unsigned target_current = set_mode ? unsigned(std::stoi(argv[2])) : 0;
    if (set_mode) for (int finger : grip_fingers) targets[finger - 1] = target_current;
    Session session;
    session.slave = left ? 1 : 2;
    session.bus = left ? "canbus0" : "canbus1";
    DeviceHandler* handler = nullptr;
    cb::CallbackContext context{&session};
    bool opened = false, registered = false;
    int lock_fd = -1;
    auto& controller = cb::CanBusController::getInstance();
    const auto cleanup = [&] {
        if (handler) close_device_handler(handler, STARK_PROTOCOL_TYPE_CAN);
        // Removing the only registered device closes its bus inside canbus_sdk.
        if (registered) controller.unregisterDevice(cb::DeviceType::REVO1_HAND, session.slave, session.bus);
        else if (opened) controller.closeCanBus(session.bus);
        active = nullptr;
        if (lock_fd >= 0) close(lock_fd);
    };
    try {
        reject_running_controller();
        lock_fd = open("/home/lab/kuavo_pi05_lower/hand_force.lock", O_CREAT | O_RDWR, 0600);
        require(lock_fd >= 0, "Cannot open the hand diagnostic lock");
        require(flock(lock_fd, LOCK_EX | LOCK_NB) == 0, "Another hand diagnostic owns the lock");
        cb::ConfigParser config;
        require(config.parseFromFile("/home/lab/.config/lejuconfig/canbus_device_cofig.yaml"), "Cannot parse CAN config");
        cb::CanBusConfig bus{};
        bool found = false;
        for (const auto& candidate : config.getCanBusConfigs()) if (candidate.name == session.bus) {
            bus = candidate;
            found = candidate.enable;
        }
        require(found, "Expected CAN bus is absent or disabled");
        found = false;
        for (const auto& candidate : config.getDevices(session.bus, cb::DeviceType::REVO1_HAND)) {
            if (candidate.device_id == session.slave && !candidate.ignore) found = true;
        }
        require(found, "Expected Revo1 hand is absent or disabled");
        controller.init();
        require(controller.openCanBus(bus.name, bus.type, bus.bitrate).has_value(), "Cannot open CAN adapter");
        opened = true;
        cb::CallbackParams callbacks{on_frame, &context, nullptr, nullptr};
        // Only one hand receiver is registered; its SDK RX callback filters exact response IDs.
        cb::DeviceInfo device("pi05_force_read_only", cb::DeviceType::REVO1_HAND,
                               session.slave, [](uint32_t, uint32_t) { return true; });
        require(controller.registerDevice(device, session.bus, callbacks).has_value(), "Cannot register hand receiver");
        registered = true;
        active = &session;
        init_logging(LOG_LEVEL_DEBUG);
        set_can_tx_callback(send_read);
        set_can_rx_callback(receive_read);
        // Match the existing Revo1CanDexhand factory: infer hardware from real identity.
        handler = init_device_handler(STARK_PROTOCOL_TYPE_CAN, 1);
        require(handler != nullptr, "Cannot initialize SDK handler");
        CDeviceInfo* info = stark_get_device_info(handler, session.slave);
        require(info != nullptr, "Device identity read failed; no force request attempted");
        const int sku = static_cast<int>(info->sku_type);
        const bool supported = info->hardware_type == STARK_HARDWARE_TYPE_REVO1_ADVANCED &&
                               std::strcmp(info->firmware_version, "0.0.14.C") == 0;
        std::printf("HAND_IDENTITY side=%s sku=%d hardware=%d firmware=%s\n", side,
                    sku, static_cast<int>(info->hardware_type), info->firmware_version);
        free_device_info(info);
        require(sku == (left ? 2 : 1), "Readback hand side differs from configuration");
        require(supported, "This tool was verified only for Revo1Advanced firmware 0.0.14.C");
        // Force-level and maximum-current getters are not applicable to this CAN
        // backend. Do not call unsupported APIs before capability capture.
        if (std::strcmp(argv[1], "--capture-write") == 0) {
            session.capture_write = true;
            session.capture_operation = "known_position_control";
            std::printf("CAPTURE_BEGIN operation=%s\n", session.capture_operation.load());
            const uint16_t positions[6] = {};
            stark_set_finger_positions(handler, session.slave, positions, 6);
            std::this_thread::sleep_for(750ms);
            session.capture_operation = "protected_current_600";
            std::printf("CAPTURE_BEGIN operation=%s\n", session.capture_operation.load());
            stark_set_finger_protected_current(handler, session.slave, STARK_FINGER_ID_THUMB, 600);
            std::this_thread::sleep_for(750ms);
            session.capture_operation = "turbo_enable";
            std::printf("CAPTURE_BEGIN operation=%s\n", session.capture_operation.load());
            stark_set_turbo_mode_enabled(handler, session.slave, true);
            std::this_thread::sleep_for(750ms);
            session.capture_operation = "current_control";
            std::printf("CAPTURE_BEGIN operation=%s\n", session.capture_operation.load());
            const int16_t currents[6] = {300, 0, 300, 300, 300, 300};
            stark_set_finger_currents(handler, session.slave, currents, 6);
            std::this_thread::sleep_for(1500ms);
            std::puts("CAPTURE_COMPLETE applied_to_device=false");
            // Keep capturing while the SDK closes, including any delayed tasks.
            cleanup();
            return 0;
        }
        bool valid = true;
        std::array<unsigned, 6> original{};
        for (int finger = 1; finger <= 6; ++finger) {
            unsigned before_tx = session.tx, before_rx = session.rx;
            const auto protected_current = stark_get_finger_protected_current(handler,
                                 session.slave, static_cast<StarkFingerId>(finger));
            bool protected_valid = session.tx > before_tx && session.rx > before_rx && protected_current > 0;
            original[finger - 1] = protected_current;
            valid = valid && protected_valid;
            std::printf("CURRENT_LIMIT side=%s finger=%d protected_mA=%u protected_verified=%s\n",
                        side, finger,
                        protected_current, protected_valid ? "true" : "false");
        }
        auto* status = stark_get_motor_status(handler, session.slave);
        bool idle = status != nullptr;
        if (status) {
            for (int finger = 0; finger < 6; ++finger) {
                std::printf("MOTOR_STATE side=%s finger=%d position=%u current=%d state=%u\n",
                            side, finger + 1, status->positions[finger],
                            status->currents[finger], status->states[finger]);
                idle = idle && status->states[finger] == 0;
            }
            free_motor_status_data(status);
        } else std::printf("MOTOR_STATE side=%s available=false\n", side);
        if (set_mode || restore_mode) {
            require(valid && idle, "Current limits must be verified and all fingers idle before configuration");
            for (int finger : {1, 3, 4, 5, 6})
                require(original[finger - 1] == 500 || original[finger - 1] == 600,
                        "Unexpected starting current limit; preserve unknown configuration");
            std::vector<int> changed;
            try {
                for (int finger : grip_fingers) if (original[finger - 1] != targets[finger - 1]) {
                    changed.push_back(finger);
                    write_verified(handler, session, finger, targets[finger - 1]);
                }
                require(read_protection(handler, session, 2) == original[1], "AUX current changed unexpectedly");
            } catch (...) {
                for (auto it = changed.rbegin(); it != changed.rend(); ++it) {
                    try { write_verified(handler, session, *it, original[*it - 1]); }
                    catch (const std::exception& e) { std::fprintf(stderr, "ROLLBACK_FAILED finger=%d error=%s\n", *it, e.what()); }
                }
                throw;
            }
            if (set_mode)
                std::printf("HAND_FORCE_VERIFIED side=%s grip_current_ma=%u aux_current_ma=%u\n",
                            side, target_current, original[1]);
            else
                std::printf("HAND_FORCE_RESTORED side=%s grip_current_ma=%s aux_current_ma=%u\n",
                            side, argv[2], original[1]);
        }
        cleanup();
        return valid ? 0 : 4;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "READ_ONLY_INSPECTION_FAILED error=%s\n", e.what());
        cleanup();
        return 3;
    }
}
