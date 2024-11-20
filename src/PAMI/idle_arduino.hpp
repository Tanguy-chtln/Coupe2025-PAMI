#ifndef _IDLE_ARDUINO_HPP_
#define _IDLE_ARDUINO_HPP_

#include <chrono>
#include <condition_variable>
#include <cstring>
#include <functional>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

class String {
  private:
    char *data;
    size_t _length;

    void allocate_and_copy(const char *src) {
        if (src) {
            _length = std::strlen(src);
            data = new char[_length + 1];
            std::strcpy(data, src);
        } else {
            _length = 0;
            data = new char[1]{'\0'};
        }
    }

  public:
    String() : data(new char[1]{'\0'}), _length(0) {}

    String(const char *str) { allocate_and_copy(str); }

    String(const String &other) { allocate_and_copy(other.data); }

    String(String &&other) noexcept : data(other.data), _length(other._length) {
        other.data = nullptr;
        other._length = 0;
    }

    ~String() { delete[] data; }

    String &operator=(const String &other) {
        if (this != &other) {
            delete[] data;
            allocate_and_copy(other.data);
        }
        return *this;
    }

    String &operator=(String &&other) noexcept {
        if (this != &other) {
            delete[] data;
            data = other.data;
            _length = other._length;

            other.data = nullptr;
            other._length = 0;
        }
        return *this;
    }

    String operator+(const String &other) const {
        size_t new_length = _length + other._length;
        char *new_data = new char[new_length + 1];

        std::strcpy(new_data, data);
        std::strcat(new_data, other.data);

        String result(new_data);
        delete[] new_data;
        return result;
    }

    String &operator+=(const String &other) {
        *this = *this + other;
        return *this;
    }

    bool operator==(const String &other) const { return std::strcmp(data, other.data) == 0; }

    bool operator!=(const String &other) const { return !(*this == other); }

    char &operator[](size_t index) {
        if (index >= _length)
            throw std::out_of_range("Index out of range");
        return data[index];
    }

    const char &operator[](size_t index) const {
        if (index >= _length)
            throw std::out_of_range("Index out of range");
        return data[index];
    }

    size_t length() const { return _length; }

    const char *c_str() const { return data; }

    friend std::ostream &operator<<(std::ostream &os, const String &str) {
        os << str.data;
        return os;
    }

    String &operator+=(char ch) {
        char str[2] = {ch, '\0'};
        *this += String(str);
        return *this;
    }
};

#define GPT_TIMER 0
std::mutex genMtx;
typedef struct {
    void *arg;
} timer_callback_args_t;

typedef struct {
    void *_;
} timer_source_div_t;

using GPTimerCbk_f = std::function<void(timer_callback_args_t *)>;
using Irq_f = void *;
#define LOW 0
#define HIGH 1
typedef enum { TIMER_MODE_PERIODIC } timer_mode_t;

void digitalWrite(int, int) {}

class Serial_t {
  public:
    static void begin(const int baudRate) { std::cout << "Serial started at " << baudRate << " baud" << std::endl; }

    static void println(const std::string &text) { std::cout << text << std::endl; }
    static void println(const String &text) { std::cout << text.c_str() << std::endl; }
    static void println(const char *text) { std::cout << text << std::endl; }

    static char read() { return '0'; }

    static bool available() { return true; }
};

class FspTimer {
  private:
    bool isRunning;
    bool isClosed;

    void fixedFrequencyTask(double frequency, GPTimerCbk_f task) {
        using namespace std::chrono;
        auto interval = duration<double>(1.0 / frequency);
        auto next_time = steady_clock::now() + interval;

        std::mutex mtx;
        std::condition_variable cv;
        while (!isClosed) {
            while (isRunning) {
                task(nullptr);

                std::unique_lock<std::mutex> lock(mtx);
                cv.wait_until(lock, next_time, [] { return false; });
                next_time += interval;
            }
        }
    }

    void launch_callback(GPTimerCbk_f callback, double frequency) {
        std::thread([=]() { fixedFrequencyTask(frequency, callback); }).detach();
    }

  public:
    static int8_t get_available_timer(int8_t &, bool = false) { return 0; }

    static void force_use_of_pwm_reserved_timer() {}

    static void release_pwm_reserved_timer(){}

    bool begin(timer_mode_t, uint8_t, uint8_t, float freq_hz, float, GPTimerCbk_f cbk = nullptr, void * = nullptr) {
        if (cbk) {
            isRunning = true;
            isClosed = false;
            launch_callback(cbk, freq_hz);
        }
        return true;
    }

    bool begin(timer_mode_t, uint8_t, uint8_t, uint32_t period, uint32_t, timer_source_div_t, GPTimerCbk_f cbk = nullptr, void * = nullptr) {
        if (cbk) {
            isRunning = true;
            isClosed = false;
            launch_callback(cbk, 1 / period);
        }
        return true;
    }

    bool setup_overflow_irq(uint8_t = 12, Irq_f = nullptr) { return true; }

    bool open() { return true; }

    bool start() { return true; }

    bool stop() {
        isRunning = false;
        return true;
    }
    bool close() {
        isClosed = false;
        return true;
    }
};

void interrupts() { genMtx.lock(); }

void noInterrupts() { genMtx.unlock(); }

auto program_start_time = std::chrono::steady_clock::now();

unsigned long millis() {
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - program_start_time);
    return static_cast<unsigned long>(elapsed.count());
}

unsigned long micros() {
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(now - program_start_time);
    return static_cast<unsigned long>(elapsed.count());
}

Serial_t Serial;

#endif // _IDLE_ARDUINO_HPP_
